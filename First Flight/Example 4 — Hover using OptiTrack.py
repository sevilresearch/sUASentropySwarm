
# Crazyflie hover GUI using OptiTrack (NatNet) position.
# - Live X/Y/Z display (from OptiTrack)
# - Takeoff/Land with user-entered altitude (meters)
# - Auto-Hold (XY) with smooth velocity servo (P control)
# - Rigid Body ID selector (e.g., CF1=1, CF2=2)
# - FIX: Convert world-frame error to CF body frame using yaw from OptiTrack
#
# Axis mapping per your environment:
#   incoming OptiTrack (x_in, y_in, z_in) --> GUI pos: (x, z, y)

import math
import threading
import time
import tkinter as tk
from tkinter import ttk

# Crazyflie API
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# OptiTrack NatNet SDK (NatNetClient.py must be available)
from NatNetClient import NatNetClient

URI = 'radio://0/80/2M'

cf = None
scf = None
mc = None
running = True
optitrack_ready = False  # becomes True after first valid RB frame

# Position updated from OptiTrack callback (meters)
pos  = {"x": 0.0, "y": 0.0, "z": 0.0}
home = {"x": None, "y": None, "z": None}  # takeoff reference

# Current yaw (rad) of the Crazyflie in the OptiTrack/world frame.
# We use this to rotate world error -> body velocities.
yaw_world = 0.0

# If your NatNet quaternion arrives as (qx, qy, qz, qw) keep "xyzw".
# If it arrives as (qw, qx, qy, qz) set to "wxyz".
Q_ORDER = "xyzw"

# ---------------- Helpers ----------------
def set_param(cf, name, value):
    try:
        cf.param.set_value(name, str(value))
    except Exception as e:
        print(f"[WARN] set {name}={value} failed: {e}")

def kalman_setup_and_reset(cf):
    set_param(cf, 'commander.enHighLevel', 1)
    set_param(cf, 'stabilizer.estimator', 2)  # 2 = Kalman
    time.sleep(0.1)
    set_param(cf, 'kalman.resetEstimation', 1)
    time.sleep(0.1)
    set_param(cf, 'kalman.resetEstimation', 0)
    time.sleep(0.6)

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

def quat_to_yaw(q):
    """
    Convert quaternion -> yaw (rad) about world up-axis.
    q is either (qx, qy, qz, qw) or (qw, qx, qy, qz) depending on Q_ORDER.
    Formula assumes right-handed coordinates; yaw extraction from quaternion:
      yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    """
    if Q_ORDER == "xyzw":
        qx, qy, qz, qw = q
    else:  # "wxyz"
        qw, qx, qy, qz = q
    # Normalize to be safe
    norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz) + 1e-12
    qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm
    # Yaw about world Z (up). This matches OptiTrack convention (z up).
    yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
    return yaw

# ------------- OptiTrack streaming -------------
def on_rigid_body_frame(rb_id, position, rotation):
    """NatNet callback: update pos{} and yaw_world from the selected RB only."""
    global pos, optitrack_ready, yaw_world
    try:
        if rb_id != selected_rb_id.get():
            return

        x_in, y_in, z_in = position

        # Axis remap (x, y, z) -> (x, z, y) per your environment
        pos['x'] = float(x_in)
        pos['y'] = float(z_in)
        pos['z'] = float(y_in)

        # Save yaw (world frame) from quaternion
        # rotation is a 4-tuple quaternion; see Q_ORDER above
        yaw_world = quat_to_yaw(rotation)

        optitrack_ready = True
    except Exception as e:
        print(f"[NatNet] parse error: {e}")

def start_optitrack():
    """Start NatNet client (receiver thread)."""
    client = NatNetClient()
    client.rigidBodyListener = on_rigid_body_frame
    # For unicast, you can set addresses here:
    # client.set_client_address("x.x.x.x")
    # client.set_server_address("x.x.x.x")
    client.run()  # non-blocking
    return client

# --------------- Flight controls ---------------
def do_takeoff():
    global mc, home
    if mc is not None:
        return
    try:
        try:
            target_alt = float(alt_entry.get())
        except ValueError:
            status_var.set("Status: Invalid altitude input")
            return

        # Capture home position at first takeoff
        if home["x"] is None:
            home["x"], home["y"], home["z"] = pos["x"], pos["y"], pos["z"]

        mc = MotionCommander(scf, default_height=target_alt)
        mc.take_off(height=target_alt, velocity=0.15)
        status_var.set(
            f"Status: Hovering at {target_alt:.2f} m "
            f"(RB {selected_rb_id.get()}, OptiTrack XY hold)"
        )
    except Exception as e:
        status_var.set(f"Status: Takeoff failed - {e}")
        mc = None

def do_land():
    global mc
    if mc is None:
        return
    try:
        mc.stop()
        mc.land()
        mc = None
        status_var.set("Status: Landed")
    except Exception as e:
        status_var.set(f"Status: Land error - {e}")
        mc = None

# --------------- Background workers ---------------
def cf_worker():
    """Connect CF, enable HL+Kalman, start OptiTrack, keep link alive."""
    global scf, cf, running
    natnet = None
    try:
        natnet = start_optitrack()

        cflib.crtp.init_drivers(enable_debug_driver=False)
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as _scf:
            scf = _scf
            cf = scf.cf
            print("[CF] Connected")

            kalman_setup_and_reset(cf)

            status_var.set("Status: Connected ( waiting OptiTrack… )")

            # Wait a bit for OptiTrack frames from the selected RB
            t0 = time.time()
            while running and not optitrack_ready and (time.time() - t0 < 3.0):
                time.sleep(0.05)

            if optitrack_ready:
                status_var.set(
                    f"Status: Connected (OptiTrack ready, RB {selected_rb_id.get()})"
                )
            else:
                status_var.set("Status: Connected (OptiTrack NOT detected)")

            while running:
                time.sleep(0.1)

    except Exception as e:
        status_var.set(f"Status: CF error - {e}")
    finally:
        try:
            do_land()
        except Exception:
            pass
        try:
            if cf:
                cf.commander.send_stop_setpoint()
        except Exception:
            pass
        print("[CF] Disconnected")

def recenter_worker():
    """
    Smooth Auto-Hold (world -> body):
    - compute XY error in world frame,
    - rotate by -yaw to body frame,
    - command small body-frame velocities toward home.
    """
    global running
    KP = 0.8       # proportional gain (m/s per meter of error)
    period = 0.1   # 10 Hz control

    while running:
        time.sleep(period)
        try:
            if mc is None or home["x"] is None:
                continue

            # If Auto-Hold is off, stop residual motion
            if auto_hold_var.get() == 0:
                mc.stop()
                continue

            # Read GUI tuning
            try:
                deadband = float(tol_entry.get())      # meters
            except ValueError:
                deadband = 0.10
            try:
                vmax = float(vel_entry.get())          # m/s
            except ValueError:
                vmax = 0.25

            # World-frame error
            ex_w = home["x"] - pos["x"]
            ey_w = home["y"] - pos["y"]
            dist = math.hypot(ex_w, ey_w)

            if dist <= deadband:
                mc.stop()
                continue

            # Rotate world error into CF body frame using yaw:
            # body = R^T * world  (R is rotation by yaw)
            c = math.cos(yaw_world)
            s = math.sin(yaw_world)
            ex_b =  c*ex_w + s*ey_w   # forward error
            ey_b = -s*ex_w + c*ey_w   # left error

            # Proportional body-frame velocity toward home (saturated)
            vx_b = clamp(KP * ex_b, -vmax, vmax)  # +X forward
            vy_b = clamp(KP * ey_b, -vmax, vmax)  # +Y left

            # Command continuous body-frame velocity
            mc.start_linear_motion(vx_b, vy_b, 0.0)

        except Exception as e:
            print(f"[AutoHold] error: {e}")

# ---------------- GUI ----------------
def update_gui():
    """Update X/Y/Z labels and XY distance."""
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

    root.after(100, update_gui)  # ~10 Hz

def on_close():
    global running
    running = False
    try:
        do_land()
    except Exception:
        pass
    root.destroy()

# ---------- Build Tkinter GUI ----------
root = tk.Tk()
root.title("Crazyflie — OptiTrack Live Position + Auto-Hold")

main = ttk.Frame(root, padding=12)
main.grid(sticky="nsew")

ttk.Label(main, text="Crazyflie — Live Pose (OptiTrack)",
          font=("Segoe UI", 14, "bold")).grid(row=0, column=0, columnspan=4,
                                              pady=(0, 8), sticky="w")

# Live labels
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

# Altitude input + buttons
ttk.Label(main, text="Target Altitude (m):").grid(row=6, column=0, sticky="e", padx=4)
alt_entry = ttk.Entry(main, width=7)
alt_entry.insert(0, "0.5")
alt_entry.grid(row=6, column=1, sticky="w")

btn_takeoff = ttk.Button(main, text="Takeoff", command=do_takeoff)
btn_land    = ttk.Button(main, text="Land", command=do_land)
btn_takeoff.grid(row=6, column=2, padx=(12, 8), pady=4, sticky="w")
btn_land.grid(row=6, column=3, padx=(0, 8), pady=4, sticky="w")

# Rigid Body selector (CF1/CF2 etc.)
ttk.Label(main, text="Rigid Body ID:").grid(row=7, column=0, sticky="e", padx=4)
selected_rb_id = tk.IntVar(value=1)  # default RB 1 (CF1)
rb_selector = ttk.Combobox(main, textvariable=selected_rb_id, values=[1, 2],
                           width=5, state="readonly")
rb_selector.grid(row=7, column=1, sticky="w")
    
# Auto-hold controls
auto_hold_var = tk.IntVar(value=1)  # default ON
ttk.Checkbutton(main, text="Auto-Hold (XY recenter)", variable=auto_hold_var)\
   .grid(row=8, column=0, columnspan=2, sticky="w", pady=(8, 2))

ttk.Label(main, text="Tolerance (m):").grid(row=9, column=0, sticky="e", padx=4)
tol_entry = ttk.Entry(main, width=7)
tol_entry.insert(0, "0.10")
tol_entry.grid(row=9, column=1, sticky="w")

ttk.Label(main, text="Nudge velocity (m/s):").grid(row=9, column=2, sticky="e", padx=4)
vel_entry = ttk.Entry(main, width=7)
vel_entry.insert(0, "0.25")
vel_entry.grid(row=9, column=3, sticky="w")

# Start worker threads
t_cf = threading.Thread(target=cf_worker, daemon=True)
t_cf.start()

t_rc = threading.Thread(target=recenter_worker, daemon=True)
t_rc.start()

# GUI update loop + close handler
root.after(200, update_gui)
root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
