
# GUI to display Crazyflie Flow (x,y,z) and hover with user-input altitude.
# Added: Auto-Hold (recenter) to keep XY within a tolerance of takeoff point.


import math
import threading
import time
import tkinter as tk
from tkinter import ttk

# Crazyflie Python API
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

# Radio link (update if your URI is different)
URI = "radio://0/80/2M/E7E7E7E701"

# Globals for state
cf = None       # Crazyflie instance
scf = None      # SyncCrazyflie handle
mc = None       # MotionCommander instance (high-level control)
logconf = None  # Logging config
running = True  # Flag to stop threads when GUI closes

# Position state (meters), updated by log callback
pos = {"x": 0.0, "y": 0.0, "z": 0.0}
# Home position (where the CF took off from), used for drift correction
home = {"x": None, "y": None, "z": None}

# ---------------- Helpers ----------------
def set_param(cf, name, value):
    """Safely set a parameter on the Crazyflie (with error handling)."""
    try:
        cf.param.set_value(name, str(value))
    except Exception as e:
        print(f"[WARN] set {name}={value} failed: {e}")

def kalman_setup_and_reset(cf):
    """
    Enable high-level commander and Kalman estimator, then reset Kalman.
    This is required for Flow Deck hover to work.
    """
    set_param(cf, 'commander.enHighLevel', 1)   # enable HL commander
    set_param(cf, 'stabilizer.estimator', 2)    # 2 = Kalman filter
    time.sleep(0.1)
    set_param(cf, 'kalman.resetEstimation', 1)  # reset = 1
    time.sleep(0.1)
    set_param(cf, 'kalman.resetEstimation', 0)  # reset = 0
    time.sleep(0.6)  # wait for estimator to settle

def start_logging(cf, on_data, period_ms=50):
    """
    Subscribe to estimator logs (x,y,z). Callback on_data is called with new values.
    Default period: 50 ms (20 Hz).
    """
    global logconf
    logconf = LogConfig(name='State', period_in_ms=period_ms)
    logconf.add_variable('stateEstimate.x', 'float')
    logconf.add_variable('stateEstimate.y', 'float')
    logconf.add_variable('stateEstimate.z', 'float')

    def _data_cb(timestamp, data, logconf):
        on_data(data)

    def _err_cb(logconf, msg):
        print(f"[LOG ERR] {logconf.name}: {msg}")

    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(_data_cb)
    logconf.error_cb.add_callback(_err_cb)
    logconf.start()

def stop_logging():
    """Stop the log configuration if active."""
    global logconf
    if logconf is not None:
        try:
            logconf.stop()
        except Exception:
            pass
        logconf = None

# Log callback: update position variables
def on_log_data(data):
    pos["x"] = float(data.get('stateEstimate.x', pos["x"]))
    pos["y"] = float(data.get('stateEstimate.y', pos["y"]))
    pos["z"] = float(data.get('stateEstimate.z', pos["z"]))

# --------------- Flight controls ---------------
def do_takeoff():
    """Takeoff to the altitude typed in the GUI input box."""
    global mc, home
    if mc is not None:
        return
    try:
        # Read altitude input from GUI (float in meters)
        try:
            target_alt = float(alt_entry.get())
        except ValueError:
            status_var.set("Status: Invalid altitude input")
            return

        # Save home position at first takeoff
        if home["x"] is None:
            home["x"], home["y"], home["z"] = pos["x"], pos["y"], pos["z"]

        # Create MotionCommander for high-level moves
        mc = MotionCommander(scf, default_height=target_alt)
        mc.take_off(height=target_alt, velocity=0.25)  # explicit takeoff
        status_var.set(f"Status: Hovering at {target_alt:.2f} m")
    except Exception as e:
        status_var.set(f"Status: Takeoff failed - {e}")
        mc = None

def do_land():
    """Land gently and release MotionCommander."""
    global mc
    if mc is None:
        return
    try:
        mc.land()
        mc = None
        status_var.set("Status: Landed")
    except Exception as e:
        status_var.set(f"Status: Land error - {e}")
        mc = None

# --------------- Background workers ---------------
def cf_worker():
    """
    Background thread: connects to Crazyflie, sets up Kalman + logging,
    and keeps connection alive while GUI runs.
    """
    global scf, cf, running
    try:
        cflib.crtp.init_drivers(enable_debug_driver=False)
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as _scf:
            scf = _scf
            cf = scf.cf
            print("[CF] Connected")

            kalman_setup_and_reset(cf)          # enable HL commander + Kalman
            start_logging(cf, on_log_data, 50)  # log x,y,z

            status_var.set("Status: Connected (READY)")
            while running:
                time.sleep(0.1)

    except Exception as e:
        status_var.set(f"Status: CF error - {e}")
    finally:
        try:
            do_land()
        except Exception:
            pass
        stop_logging()
        try:
            if cf:
                cf.commander.send_stop_setpoint()
        except Exception:
            pass
        print("[CF] Disconnected")

def recenter_worker():
    """
    Background thread: every second, checks XY drift from home.
    If drift > tolerance, commands a small nudge back toward home.
    Parameters are read from GUI entries (tolerance, nudge step, velocity).
    """
    global running
    while running:
        time.sleep(1.0)  # check ~1 Hz
        try:
            if mc is None:
                continue
            if auto_hold_var.get() == 0:
                continue
            if home["x"] is None:
                continue

            # Read tolerance/nudge/velocity from GUI (with defaults)
            try:
                tol = float(tol_entry.get())
            except ValueError:
                tol = 0.10
            try:
                nudge = float(nudge_entry.get())
            except ValueError:
                nudge = 0.05
            try:
                v = float(vel_entry.get())
            except ValueError:
                v = 0.25

            # Compute offset from home
            dx = home["x"] - pos["x"]
            dy = home["y"] - pos["y"]
            dist = math.hypot(dx, dy)

            if dist > tol:
                # Direction unit vector back to home
                ux = dx / (dist + 1e-9)
                uy = dy / (dist + 1e-9)
                # Limit correction step
                step_x = ux * nudge
                step_y = uy * nudge
                # Send correction move (blocking, small step)
                mc.move_distance(step_x, step_y, 0.0, velocity=v)
        except Exception as e:
            print(f"[AutoHold] error: {e}")

# ---------------- GUI ----------------
def update_gui():
    """Periodic update of GUI labels with live x,y,z and drift distance."""
    if home["x"] is not None:
        dx = pos["x"] - home["x"]
        dy = pos["y"] - home["y"]
        dist_xy = math.hypot(dx, dy)
    else:
        dist_xy = 0.0

    x_var.set(f"X: {pos['x']:+.3f} m")
    y_var.set(f"Y: {pos['y']:+.3f} m")
    z_var.set(f"Z: {pos['z']:+.3f} m")
    d_var.set(f"XY dist from takeoff: {dist_xy:.3f} m")

    root.after(100, update_gui)  # refresh ~10 Hz

def on_close():
    """Handler when GUI window is closed: stop threads and land safely."""
    global running
    running = False
    try:
        do_land()
    except Exception:
        pass
    root.destroy()

# Build Tkinter GUI
root = tk.Tk()
root.title("Crazyflie — Live Position")

main = ttk.Frame(root, padding=12)
main.grid(sticky="nsew")

# Title
ttk.Label(main, text="Crazyflie — Live Pose",
          font=("Segoe UI", 14, "bold")).grid(row=0, column=0, columnspan=4, pady=(0, 8), sticky="w")

# Live state variables
x_var = tk.StringVar(value="X: +0.000 m")
y_var = tk.StringVar(value="Y: +0.000 m")
z_var = tk.StringVar(value="Z: +0.000 m")
d_var = tk.StringVar(value="XY dist from takeoff: 0.000 m")
status_var = tk.StringVar(value="Status: Connecting…")

ttk.Label(main, textvariable=x_var, font=("Consolas", 12)).grid(row=1, column=0, sticky="w")
ttk.Label(main, textvariable=y_var, font=("Consolas", 12)).grid(row=2, column=0, sticky="w")
ttk.Label(main, textvariable=z_var, font=("Consolas", 12)).grid(row=3, column=0, sticky="w")
ttk.Label(main, textvariable=d_var, font=("Consolas", 12)).grid(row=4, column=0, pady=(4, 8), sticky="w")
ttk.Label(main, textvariable=status_var).grid(row=5, column=0, columnspan=4, pady=(4, 8), sticky="w")

# Altitude input box
ttk.Label(main, text="Target Altitude (m):").grid(row=6, column=0, sticky="e", padx=4)
alt_entry = ttk.Entry(main, width=7)
alt_entry.insert(0, "0.5")  # default 0.5 m
alt_entry.grid(row=6, column=1, sticky="w")

# Buttons
btn_takeoff = ttk.Button(main, text="Takeoff", command=do_takeoff)
btn_land    = ttk.Button(main, text="Land", command=do_land)
btn_takeoff.grid(row=6, column=2, padx=(12, 8), pady=4, sticky="w")
btn_land.grid(row=6, column=3, padx=(0, 8), pady=4, sticky="w")

# Auto-hold controls
auto_hold_var = tk.IntVar(value=1)  # default ON
ttk.Checkbutton(main, text="Auto-Hold (XY recenter)", variable=auto_hold_var).grid(row=7, column=0, columnspan=2, sticky="w", pady=(8, 2))

# Tuning inputs for auto-hold
ttk.Label(main, text="Tolerance (m):").grid(row=8, column=0, sticky="e", padx=4)
tol_entry = ttk.Entry(main, width=7)
tol_entry.insert(0, "0.10")  # 10 cm tolerance
tol_entry.grid(row=8, column=1, sticky="w")

ttk.Label(main, text="Nudge step (m):").grid(row=8, column=2, sticky="e", padx=4)
nudge_entry = ttk.Entry(main, width=7)
nudge_entry.insert(0, "0.05")  # 5 cm correction step
nudge_entry.grid(row=8, column=3, sticky="w")

ttk.Label(main, text="Nudge velocity (m/s):").grid(row=9, column=0, sticky="e", padx=4)
vel_entry = ttk.Entry(main, width=7)
vel_entry.insert(0, "0.25")  # 0.25 m/s correction velocity
vel_entry.grid(row=9, column=1, sticky="w")

# Start worker threads: CF connection + recenter logic
t_cf = threading.Thread(target=cf_worker, daemon=True)
t_cf.start()

t_rc = threading.Thread(target=recenter_worker, daemon=True)
t_rc.start()

# Periodic GUI update
root.after(200, update_gui)
root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
