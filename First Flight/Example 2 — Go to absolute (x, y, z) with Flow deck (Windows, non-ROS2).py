
# Move the Crazyflie to an ABSOLUTE (x, y, z) setpoint using Flow-deck pose.
# GUI similar to the hover app: shows live pose, lets enter a target (x,y,z),
# and executes a safe, stepped trajectory until within tolerance.
#
# Key ideas
# - Keep using the Flow-based stateEstimate.{x,y,z} as local world frame.
#   The "home" is captured at first takeoff and used only for info; the absolute
#   target coordinates are the estimator coordinates (meters) at runtime.
#
# Safety notes
# - Always test low and in a clear volume. Start with small steps (<= 0.2 m)
#   and low velocity (<= 0.25 m/s). Increase only after validating tracking.
# - If you see poor tracking or drift, land immediately and re-calibrate / reset
#   the Kalman estimator.

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

# === Radio link (update if needed) ===
URI = 'radio://0/80/2M/E7E7E7E701'

# === Global state ===
cf = None          # Crazyflie instance
scf = None         # SyncCrazyflie handle
mc = None          # MotionCommander (active while airborne)
logconf = None     # Log configuration
running = True     # App running flag

# Pose (meters), updated by logger
pos = {"x": 0.0, "y": 0.0, "z": 0.0}
# Home pose captured on first takeoff (for display only)
home = {"x": None, "y": None, "z": None}

# Target setpoint (absolute, in estimator frame)
target = {"x": 0.0, "y": 0.0, "z": 0.5}

# Motion thread control
_go_thread = None
_go_running = False

# --------------- Helpers ---------------
def set_param(cf, name, value):
    try:
        cf.param.set_value(name, str(value))
    except Exception as e:
        print(f"[WARN] set {name}={value} failed: {e}")


def kalman_setup_and_reset(cf):
    """Enable HL commander + Kalman estimator and reset the filter."""
    set_param(cf, 'commander.enHighLevel', 1)  # enable high-level
    set_param(cf, 'stabilizer.estimator', 2)   # 2 = Kalman
    time.sleep(0.1)
    set_param(cf, 'kalman.resetEstimation', 1)
    time.sleep(0.1)
    set_param(cf, 'kalman.resetEstimation', 0)
    time.sleep(0.6)  # let it settle


def start_logging(cf, on_data, period_ms=50):
    """Subscribe to stateEstimate.{x,y,z} at ~20 Hz by default."""
    global logconf
    logconf = LogConfig(name='State', period_in_ms=period_ms)
    logconf.add_variable('stateEstimate.x', 'float')
    logconf.add_variable('stateEstimate.y', 'float')
    logconf.add_variable('stateEstimate.z', 'float')

    def _data_cb(timestamp, data, _):
        on_data(data)

    def _err_cb(logconf, msg):
        print(f"[LOG ERR] {logconf.name}: {msg}")

    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(_data_cb)
    logconf.error_cb.add_callback(_err_cb)
    logconf.start()


def stop_logging():
    global logconf
    if logconf is not None:
        try:
            logconf.stop()
        except Exception:
            pass
        logconf = None


def on_log_data(data):
    pos["x"] = float(data.get('stateEstimate.x', pos["x"]))
    pos["y"] = float(data.get('stateEstimate.y', pos["y"]))
    pos["z"] = float(data.get('stateEstimate.z', pos["z"]))

# --------------- Flight controls ---------------

def do_takeoff():
    """Take off to the altitude in the Z entry box."""
    global mc, home
    if mc is not None:
        return
    try:
        try:
            z = float(z_entry.get())
        except ValueError:
            status_var.set("Status: Invalid Z (meters)")
            return

        if home["x"] is None:
            home.update(x=pos["x"], y=pos["y"], z=pos["z"])  # capture home once

        mc = MotionCommander(scf, default_height=z)
        mc.take_off(height=z, velocity=0.25)
        status_var.set(f"Status: Airborne at {z:.2f} m")
    except Exception as e:
        status_var.set(f"Status: Takeoff failed — {e}")
        mc = None


def do_land():
    """Land and release MotionCommander."""
    global mc
    if mc is None:
        return
    try:
        mc.land()
        mc = None
        status_var.set("Status: Landed")
    except Exception as e:
        status_var.set(f"Status: Land error — {e}")
        mc = None


# --------------- Setpoint motion ---------------

def _step_towards(dx, dy, dz, v):
    """Move a single small relative step with MotionCommander."""
    mc.move_distance(dx, dy, dz, velocity=v)


def _go_loop():
    """Background loop: walk the CF toward the absolute (x,y,z) target."""
    global _go_running
    try:
        tol = _safe_float(tol_entry.get(), 0.08)        # arrival radius (m)
        step = _safe_float(step_entry.get(), 0.20)      # max step length (m)
        v = _safe_float(vel_entry.get(), 0.25)          # m/s
        max_time = _safe_float(timeout_entry.get(), 25) # seconds
        z_safety_min = _safe_float(zmin_entry.get(), 0.10)
        z_safety_max = _safe_float(zmax_entry.get(), 2.0)

        t0 = time.time()
        while _go_running:
            # Compute error to target in absolute frame
            ex = target["x"] - pos["x"]
            ey = target["y"] - pos["y"]
            ez = target["z"] - pos["z"]
            dist = math.sqrt(ex*ex + ey*ey + ez*ez)

            # Update GUI error labels
            err_var.set(f"Err: dx={ex:+.3f}, dy={ey:+.3f}, dz={ez:+.3f} m  | |e|={dist:.3f}")

            # Arrival check
            if dist <= tol:
                status_var.set("Status: Arrived (within tolerance)")
                break

            # Timeout check
            if (time.time() - t0) > max_time:
                status_var.set("Status: Motion timeout — stopping")
                break

            # Safety: keep Z inside bounds (command clipping)
            next_z = pos["z"] + max(min(ez, step), -step)
            if next_z < z_safety_min:
                ez_cmd = z_safety_min - pos["z"]
            elif next_z > z_safety_max:
                ez_cmd = z_safety_max - pos["z"]
            else:
                # Compute a bounded step toward the target vector
                if dist > step:
                    # Take a step of length = step along the error direction
                    ux, uy, uz = ex/dist, ey/dist, ez/dist
                    ex_cmd, ey_cmd, ez_cmd = ux*step, uy*step, uz*step
                else:
                    # Last partial step
                    ex_cmd, ey_cmd, ez_cmd = ex, ey, ez

            # Execute one step and loop
            try:
                _step_towards(ex_cmd, ey_cmd, ez_cmd, v)
            except Exception as e:
                status_var.set(f"Status: move error — {e}")
                break

            # Small pause lets logs catch up; MotionCommander is blocking
            time.sleep(0.02)

    finally:
        _go_running = False
        go_btn.config(state=tk.NORMAL)
        stop_btn.config(state=tk.DISABLED)


def _safe_float(s, default):
    try:
        return float(s)
    except Exception:
        return default


def start_go():
    """Read target (x,y,z) from GUI and start the go-thread."""
    global _go_thread, _go_running
    if mc is None:
        status_var.set("Status: Take off first")
        return

    # Read target setpoint
    try:
        target["x"] = float(x_entry.get())
        target["y"] = float(y_entry.get())
        target["z"] = float(z_entry.get())  # also used by takeoff
    except ValueError:
        status_var.set("Status: Invalid target (x,y,z)")
        return

    # Arm the worker
    if _go_running:
        return
    _go_running = True
    _go_thread = threading.Thread(target=_go_loop, daemon=True)
    _go_thread.start()
    go_btn.config(state=tk.DISABLED)
    stop_btn.config(state=tk.NORMAL)
    status_var.set("Status: Moving to target…")


def stop_go():
    """Signal the go-loop to stop after current step."""
    global _go_running
    _go_running = False
    status_var.set("Status: Motion cancelled")


def return_home():
    """Convenience: set target to home.x/y and current z, then go."""
    if home["x"] is None:
        status_var.set("Status: Home not set (take off first)")
        return
    x_entry.delete(0, tk.END); x_entry.insert(0, f"{home['x']:.3f}")
    y_entry.delete(0, tk.END); y_entry.insert(0, f"{home['y']:.3f}")
    # keep Z as-is from box
    start_go()

# --------------- Workers ---------------

def cf_worker():
    """Connect, set up estimator + logging, then idle until app closes."""
    global scf, cf
    try:
        cflib.crtp.init_drivers(enable_debug_driver=False)
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as _scf:
            scf = _scf
            cf = scf.cf
            print("[CF] Connected")

            kalman_setup_and_reset(cf)
            start_logging(cf, on_log_data, 50)

            status_var.set("Status: Connected (READY)")
            while running:
                time.sleep(0.1)
    except Exception as e:
        status_var.set(f"Status: CF error — {e}")
    finally:
        try:
            stop_go()
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

# --------------- GUI ---------------

def update_gui():
    # Position labels
    x_var.set(f"X: {pos['x']:+.3f} m")
    y_var.set(f"Y: {pos['y']:+.3f} m")
    z_var.set(f"Z: {pos['z']:+.3f} m")

    # Home/XY drift display
    if home["x"] is not None:
        dx = pos["x"] - home["x"]
        dy = pos["y"] - home["y"]
        d_xy = math.hypot(dx, dy)
    else:
        d_xy = 0.0
    d_var.set(f"XY dist from takeoff: {d_xy:.3f} m")

    # Distance to target
    ex = target["x"] - pos["x"]
    ey = target["y"] - pos["y"]
    ez = target["z"] - pos["z"]
    dist = math.sqrt(ex*ex + ey*ey + ez*ez)
    tgt_var.set(f"Target: ({target['x']:.2f}, {target['y']:.2f}, {target['z']:.2f}) m | |e|={dist:.3f} m")

    root.after(100, update_gui)


def on_close():
    global running
    running = False
    try:
        stop_go()
        do_land()
    except Exception:
        pass
    root.destroy()

# Build Tkinter GUI
root = tk.Tk()
root.title("Crazyflie — Go To (x,y,z) — Flow")

main = ttk.Frame(root, padding=12)
main.grid(sticky="nsew")

# Title
ttk.Label(main, text="Crazyflie — XYZ Setpoint Controller",
          font=("Segoe UI", 14, "bold")).grid(row=0, column=0, columnspan=6, pady=(0, 8), sticky="w")

# Live state
x_var = tk.StringVar(value="X: +0.000 m")
y_var = tk.StringVar(value="Y: +0.000 m")
z_var = tk.StringVar(value="Z: +0.000 m")
d_var = tk.StringVar(value="XY dist from takeoff: 0.000 m")
status_var = tk.StringVar(value="Status: Connecting…")
tgt_var = tk.StringVar(value="Target: (0.00, 0.00, 0.50) m | |e|=0.000 m")
err_var = tk.StringVar(value="Err: dx=+0.000, dy=+0.000, dz=+0.000 m  | |e|=0.000")

row = 1
for var in (x_var, y_var, z_var, d_var, tgt_var, err_var):
    ttk.Label(main, textvariable=var, font=("Consolas", 11)).grid(row=row, column=0, columnspan=6, sticky="w")
    row += 1

# Setpoint entries
row += 1
_tt = ttk.Label(main, text="Target X/Y/Z (m):"); _tt.grid(row=row, column=0, sticky="e", padx=4)
x_entry = ttk.Entry(main, width=8); x_entry.insert(0, "0.00"); x_entry.grid(row=row, column=1, sticky="w")
y_entry = ttk.Entry(main, width=8); y_entry.insert(0, "0.00"); y_entry.grid(row=row, column=2, sticky="w")
z_entry = ttk.Entry(main, width=8); z_entry.insert(0, "0.50"); z_entry.grid(row=row, column=3, sticky="w")

# Tuning
row += 1
_t1 = ttk.Label(main, text="Step (m):"); _t1.grid(row=row, column=0, sticky="e", padx=4)
step_entry = ttk.Entry(main, width=8); step_entry.insert(0, "0.20"); step_entry.grid(row=row, column=1, sticky="w")
_t2 = ttk.Label(main, text="Velocity (m/s):"); _t2.grid(row=row, column=2, sticky="e", padx=4)
vel_entry = ttk.Entry(main, width=8); vel_entry.insert(0, "0.25"); vel_entry.grid(row=row, column=3, sticky="w")
_t3 = ttk.Label(main, text="Tolerance (m):"); _t3.grid(row=row, column=4, sticky="e", padx=4)
tol_entry = ttk.Entry(main, width=8); tol_entry.insert(0, "0.08"); tol_entry.grid(row=row, column=5, sticky="w")

row += 1
_t4 = ttk.Label(main, text="Timeout (s):"); _t4.grid(row=row, column=0, sticky="e", padx=4)
timeout_entry = ttk.Entry(main, width=8); timeout_entry.insert(0, "25"); timeout_entry.grid(row=row, column=1, sticky="w")
_t5 = ttk.Label(main, text="Z min/max (m):"); _t5.grid(row=row, column=2, sticky="e", padx=4)
zmin_entry = ttk.Entry(main, width=6); zmin_entry.insert(0, "0.10"); zmin_entry.grid(row=row, column=3, sticky="w")
zmax_entry = ttk.Entry(main, width=6); zmax_entry.insert(0, "2.00"); zmax_entry.grid(row=row, column=4, sticky="w")

# Buttons
row += 1
btn_takeoff = ttk.Button(main, text="Takeoff", command=do_takeoff)
btn_land    = ttk.Button(main, text="Land", command=do_land)
go_btn      = ttk.Button(main, text="Go To (x,y,z)", command=start_go)
stop_btn    = ttk.Button(main, text="Stop Motion", command=stop_go, state=tk.DISABLED)
ret_btn     = ttk.Button(main, text="Return Home (XY)", command=return_home)

btn_takeoff.grid(row=row, column=0, padx=(8, 8), pady=8, sticky="w")
btn_land.grid(row=row, column=1, padx=(0, 8), pady=8, sticky="w")
go_btn.grid(row=row, column=2, padx=(0, 8), pady=8, sticky="w")
stop_btn.grid(row=row, column=3, padx=(0, 8), pady=8, sticky="w")
ret_btn.grid(row=row, column=4, padx=(0, 8), pady=8, sticky="w")

# Status line
row += 1
ttk.Label(main, textvariable=status_var).grid(row=row, column=0, columnspan=6, sticky="w")

# Start worker threads
_t = threading.Thread(target=cf_worker, daemon=True)
_t.start()

# GUI update tick
root.after(200, update_gui)
root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
