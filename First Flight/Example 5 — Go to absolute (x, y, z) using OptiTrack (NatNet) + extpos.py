
import math
import threading
import time
import csv, os, datetime
import tkinter as tk
from tkinter import ttk

# Crazyflie Python API
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# ---- OptiTrack / NatNet ----
try:
    from NatNetClient import NatNetClient
except Exception as e:
    raise SystemExit("NatNetClient.py missing or incompatible: %s" % e)

# ================= Config =================
URI = 'radio://0/80/2M/E7E7E7E701'   # default; can be switched in GUI
# You can select by name or by numeric ID (set one, leave the other None)
RIGID_BODY_NAME = 'CF1'              # e.g., 'CF1' or 'CF2' (updated by GUI)
RIGID_BODY_ID   = None               # e.g., 1 or 2 (overrides name if set)

# Preset profiles to quickly switch which Crazyflie to move
CF_PROFILES = {
    'CF1': {'uri': 'radio://0/80/2M/E7E7E7E701', 'rb_name': 'CF1', 'rb_id': None},
    'CF2': {'uri': 'radio://0/80/2M/E7E7E7E702', 'rb_name': 'CF2', 'rb_id': None},
}

# Motive / NatNet networking (Z-Up, Multicast)
SERVER_IP = '192.168.15.106'         # PC running Motive
LOCAL_IP  = '192.168.15.106'         # this client NIC (same machine -> same IP)
COMMAND_PORT = 1510
DATA_PORT    = 1511
EXTPOS_RATE_HZ = 100.0               # feed Kalman at ~100 Hz

# ================= Globals =================
cf = None
scf = None
mc = None
running = True

# Latest OptiTrack pose (internal x,y,z in meters)
_pos_lock = threading.Lock()
pos = {"x": 0.0, "y": 0.0, "z": 0.0}

# Home pose captured on first takeoff (for display only)
home = {"x": None, "y": None, "z": None}

# Target setpoint (absolute, in external frame)
target = {"x": 0.0, "y": 0.0, "z": 0.5}

# Motion thread control
_go_thread = None
_go_running = False

# --------- CSV logging (added, self-contained) ---------
_log_rows = []
_log_active = False
_log_t0 = None
_log_path = None

def _log_start():
    global _log_rows, _log_active, _log_t0, _log_path
    _log_rows = []
    _log_active = True
    _log_t0 = time.time()
    _log_path = None

def _log_tick(px, py, pz):
    if not _log_active:
        return
    t = time.time() - (_log_t0 or time.time())
    ex = target['x'] - px; ey = target['y'] - py; ez = target['z'] - pz
    en = math.sqrt(ex*ex + ey*ey + ez*ez)
    _log_rows.append([t, px, py, pz, target['x'], target['y'], target['z'], ex, ey, ez, en])

def _log_stop_and_save():
    global _log_active, _log_path
    if not _log_active:
        return None
    _log_active = False
    if not _log_rows:
        return None
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    _log_path = os.path.join(os.getcwd(), f'log_run_{ts}.csv')
    try:
        with open(_log_path, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['time_s','x','y','z','target_x','target_y','target_z','err_x','err_y','err_z','err_norm'])
            w.writerows(_log_rows)
        print(f"[LOG] Saved: {_log_path}")
    except Exception as e:
        print(f"[LOG] Save error: {e}")
        _log_path = None
    return _log_path

# ================= Helpers =================

def set_param(cf, name, value):
    try:
        cf.param.set_value(name, str(value))
    except Exception as e:
        print(f"[WARN] set {name}={value} failed: {e}")


def kalman_setup_and_reset(cf):
    # Use HL commander and Kalman estimator (fed by extpos)
    set_param(cf, 'commander.enHighLevel', 1)
    set_param(cf, 'stabilizer.estimator', 2)  # 2 = Kalman
    time.sleep(0.1)
    set_param(cf, 'kalman.resetEstimation', 1)
    time.sleep(0.1)
    set_param(cf, 'kalman.resetEstimation', 0)
    time.sleep(0.6)

# OptiTrack (X,Y,Z) → internal (x,y,z) — Z-Up means identity mapping
# If you change Motive to Y-Up, swap Y/Z accordingly.
def map_opti_to_internal(X, Y, Z):
    return float(X), float(Y), float(Z)

# ================= OptiTrack =================

def _on_rigid_body_frame(rb_key, pos_tuple, rot_quat):
    """Accepts either name or ID depending on NatNet callback variant.
    rb_key can be a string name (e.g., 'CF1') or an int ID.
    """
    # Filter by ID if provided, else by name
    if isinstance(rb_key, int):
        if RIGID_BODY_ID is not None and rb_key != RIGID_BODY_ID:
            return
    else:
        if RIGID_BODY_ID is not None:
            pass  # accepting any name if ID is enforced
        elif RIGID_BODY_NAME and rb_key != RIGID_BODY_NAME:
            return
    x, y, z = map_opti_to_internal(pos_tuple[0], pos_tuple[1], pos_tuple[2])
    with _pos_lock:
        pos["x"], pos["y"], pos["z"] = x, y, z


def make_natnet_client():
    """Create a NatNetClient across different API variants and set IP/ports."""
    try:
        # Modern NatNetClient (most versions)
        c = NatNetClient(serverIP=SERVER_IP, localIP=LOCAL_IP,
                         commandPort=COMMAND_PORT, dataPort=DATA_PORT)
    except TypeError:
        try:
            # Older versions use different argument names
            c = NatNetClient(serverAddress=SERVER_IP, localAddress=LOCAL_IP)
        except TypeError:
            # Fallback: minimal init
            c = NatNetClient()

    # Best-effort: assign IPs and ports regardless of constructor signature
    for attr, val in (
        ('serverIPAddress', SERVER_IP), ('localIPAddress', LOCAL_IP),
        ('serverIP', SERVER_IP), ('localIP', LOCAL_IP),
        ('commandPort', COMMAND_PORT), ('dataPort', DATA_PORT),
    ):
        if hasattr(c, attr):
            try:
                setattr(c, attr, val)
            except Exception:
                pass
    return c
   

def optitrack_worker():
    client = make_natnet_client()
    # Wire up any supported callback names
    if hasattr(client, 'rigidBodyListener'):
        client.rigidBodyListener = _on_rigid_body_frame  # may pass (id, pos, rot) or (name, pos, rot)
    if hasattr(client, 'rigid_body_listener'):
        client.rigid_body_listener = _on_rigid_body_frame
    if hasattr(client, 'newFrameListener'):
        def frame_cb(data):
            # Fallback parser for some NatNet samples
            rbs = data.get('rigid_bodies', {})
            # Permit both name and ID keys
            key = RIGID_BODY_ID if RIGID_BODY_ID is not None else RIGID_BODY_NAME
            if key in rbs:
                p, r = rbs[key]
                _on_rigid_body_frame(key, p, r)
        client.newFrameListener = frame_cb
    try:
        if hasattr(client, 'run'):
            client.run()
        elif hasattr(client, 'start'):
            client.start()
        else:
            raise RuntimeError('NatNetClient: no run()/start() method found')
    except Exception as e:
        print('[NatNet] error:', e)
        # Some versions use start()
        if hasattr(client, 'start'):
            client.start()


def extpos_sender_worker():
    # Feed Kalman with external position and keep GUI pose fresh
    dt = 1.0 / max(1.0, EXTPOS_RATE_HZ)
    while running:
        time.sleep(dt)
        if cf is None:
            continue
        with _pos_lock:
            x, y, z = pos["x"], pos["y"], pos["z"]
        try:
            cf.extpos.send_extpos(x, y, z)
        except Exception as e:
            print("[extpos]", e)


# ================= Motion =================

def _safe_float(s, default):
    try:
        return float(s)
    except Exception:
        return default


def do_takeoff():
    global mc, home
    if mc is not None:
        return
    try:
        z = float(z_entry.get())
    except ValueError:
        status_var.set("Status: Invalid Z (meters)")
        return
    if home["x"] is None:
        with _pos_lock:
            home.update(x=pos["x"], y=pos["y"], z=pos["z"])  # capture once
    try:
        mc = MotionCommander(scf, default_height=z)
        mc.take_off(height=z, velocity=0.25)
        _log_start()  # <— start logging when we lift (added)
        status_var.set(f"Status: Airborne at {z:.2f} m (OptiTrack)")
    except Exception as e:
        mc = None
        status_var.set(f"Status: Takeoff failed — {e}")


def do_land():
    global mc
    if mc is None:
        return
    try:
        mc.land()
        mc = None
        _log_stop_and_save()  # <— save CSV on landing (added)
        status_var.set("Status: Landed")
    except Exception as e:
        mc = None
        status_var.set(f"Status: Land error — {e}")


def _step_towards(dx, dy, dz, v):
    mc.move_distance(dx, dy, dz, velocity=v)


def _go_loop():
    global _go_running
    try:
        tol = _safe_float(tol_entry.get(), 0.08)        # arrival radius (m)
        step = _safe_float(step_entry.get(), 0.20)      # max step length (m)
        v    = _safe_float(vel_entry.get(), 0.25)       # m/s
        tmax = _safe_float(timeout_entry.get(), 25)     # seconds
        zmin = _safe_float(zmin_entry.get(), 0.10)
        zmax = _safe_float(zmax_entry.get(), 2.00)

        t0 = time.time()
        while _go_running:
            with _pos_lock:
                ex = target["x"] - pos["x"]
                ey = target["y"] - pos["y"]
                ez = target["z"] - pos["z"]
                px, py, pz = pos['x'], pos['y'], pos['z']
            dist = math.sqrt(ex*ex + ey*ey + ez*ez)
            err_var.set(f"Err: dx={ex:+.3f}, dy={ey:+.3f}, dz={ez:+.3f} m  | |e|={dist:.3f}")

            # tick logger on each loop (about 50 Hz here, but GUI saves ~5 Hz too)
            _log_tick(px, py, pz)

            if dist <= tol:
                status_var.set("Status: Arrived (within tolerance)")
                break
            if (time.time() - t0) > tmax:
                status_var.set("Status: Motion timeout — stopping")
                break

            # Bounded step toward target
            if dist > step:
                ux, uy, uz = ex/dist, ey/dist, ez/dist
                dx, dy, dz = ux*step, uy*step, uz*step
            else:
                dx, dy, dz = ex, ey, ez

            # Clip commanded Z to safety window
            with _pos_lock:
                next_z = pos["z"] + dz
            if next_z < zmin:
                dz = zmin - (next_z - dz)  # bring to zmin
            elif next_z > zmax:
                dz = zmax - (next_z - dz)  # bring to zmax

            try:
                _step_towards(dx, dy, dz, v)
            except Exception as e:
                status_var.set(f"Status: move error — {e}")
                break
            time.sleep(0.02)
    finally:
        _go_running = False
        go_btn.config(state=tk.NORMAL)
        stop_btn.config(state=tk.DISABLED)


def start_go():
    global _go_thread, _go_running
    if mc is None:
        status_var.set("Status: Take off first")
        return
    try:
        target["x"] = float(x_entry.get())
        target["y"] = float(y_entry.get())
        target["z"] = float(z_entry.get())
    except ValueError:
        status_var.set("Status: Invalid target (x,y,z)")
        return
    if _go_running:
        return
    _go_running = True
    _go_thread = threading.Thread(target=_go_loop, daemon=True)
    _go_thread.start()
    status_var.set("Status: Moving to target…")


def stop_go():
    global _go_running
    _go_running = False
    status_var.set("Status: Motion cancelled")


def return_home():
    if home["x"] is None:
        status_var.set("Status: Home not set (take off first)")
        return
    x_entry.delete(0, tk.END); x_entry.insert(0, f"{home['x']:.3f}")
    y_entry.delete(0, tk.END); y_entry.insert(0, f"{home['y']:.3f}")
    # Keep current Z box
    start_go()


# ================= CF worker =================
# We allow switching between CFs at runtime. A small controller manages the link.
_cf_thread = None
_cf_stop_evt = threading.Event()


def _cf_worker_loop(uri):
    global scf, cf
    try:
        cflib.crtp.init_drivers(enable_debug_driver=False)
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as _scf:
            scf = _scf
            cf  = scf.cf
            print('[CF] Connected', uri)

            kalman_setup_and_reset(cf)
            status_var.set(f'Status: Connected (READY) — {uri}')

            while running and not _cf_stop_evt.is_set():
                time.sleep(0.1)
    except Exception as e:
        status_var.set(f"Status: CF error — {e}")
    finally:
        try:
            stop_go(); do_land()
        except Exception:
            pass
        try:
            if cf:
                cf.commander.send_stop_setpoint()
        except Exception:
            pass
        print('[CF] Disconnected')


def start_cf_link(uri):
    global _cf_thread
    _cf_stop_evt.clear()
    _cf_thread = threading.Thread(target=_cf_worker_loop, args=(uri,), daemon=True)
    _cf_thread.start()


def stop_cf_link():
    _cf_stop_evt.set()

# ================= GUI =================

def update_gui():
    with _pos_lock:
        px, py, pz = pos['x'], pos['y'], pos['z']
    x_var.set(f"X: {px:+.3f} m")
    y_var.set(f"Y: {py:+.3f} m")
    z_var.set(f"Z: {pz:+.3f} m")

    if home['x'] is not None:
        dx = px - home['x']; dy = py - home['y']
        d_xy = math.hypot(dx, dy)
    else:
        d_xy = 0.0
    d_var.set(f"XY dist from takeoff: {d_xy:.3f} m")

    ex = target['x'] - px; ey = target['y'] - py; ez = target['z'] - pz
    dist = math.sqrt(ex*ex + ey*ey + ez*ez)
    tgt_var.set(f"Target: ({target['x']:.2f}, {target['y']:.2f}, {target['z']:.2f}) m | |e|={dist:.3f} m")

    # tick logger at GUI rate (~5 Hz)
    _log_tick(px, py, pz)

    # Show last OptiTrack heartbeat time to confirm streaming before takeoff
    opti_var.set(f"OptiTrack: last frame at {last_opti_var.get()}")

    root.after(200, update_gui)


def on_close():
    global running
    running = False
    try:
        stop_go(); do_land()
    except Exception:
        pass
    _log_stop_and_save()  # <— save if still active on exit
    root.destroy()


# Build Tkinter GUI (unchanged style)
root = tk.Tk()
root.title("Crazyflie — Go To (x,y,z) — OptiTrack")

main = ttk.Frame(root, padding=12)
main.grid(sticky="nsew")

# Title
ttk.Label(main, text="Crazyflie — XYZ Setpoint Controller (OptiTrack)",
          font=("Segoe UI", 14, "bold")).grid(row=0, column=0, columnspan=8, pady=(0, 8), sticky="w")

# --- CF selector row (minimal addition) ---
row = 1
_ttcf = ttk.Label(main, text="Target CF:"); _ttcf.grid(row=row, column=0, sticky="e", padx=4)
cf_combo = ttk.Combobox(main, values=sorted(CF_PROFILES.keys()), width=8, state="readonly")
cf_combo.set('CF1')
cf_uri_var = tk.StringVar(value=CF_PROFILES['CF1']['uri'])
cf_uri_entry = ttk.Entry(main, textvariable=cf_uri_var, width=28)
cf_uri_entry.grid(row=row, column=2, columnspan=3, sticky="w")
rb_name_var = tk.StringVar(value=CF_PROFILES['CF1']['rb_name'])
rb_id_var   = tk.StringVar(value="")

def _apply_cf_profile(*_):
    name = cf_combo.get()
    prof = CF_PROFILES.get(name, CF_PROFILES['CF1'])
    cf_uri_var.set(prof['uri'])
    rb_name_var.set(prof['rb_name'] or '')
    rb_id_var.set('' if prof['rb_id'] is None else str(prof['rb_id']))
cf_combo.bind("<<ComboboxSelected>>", _apply_cf_profile)
cf_combo.grid(row=row, column=1, sticky="w")

# RB selector (either name or ID)
ttk.Label(main, text="RB name:").grid(row=row, column=5, sticky="e", padx=4)
rb_name_entry = ttk.Entry(main, textvariable=rb_name_var, width=10)
rb_name_entry.grid(row=row, column=6, sticky="w")
ttk.Label(main, text="RB id:").grid(row=row, column=7, sticky="e", padx=4)
rb_id_entry = ttk.Entry(main, textvariable=rb_id_var, width=5)
rb_id_entry.grid(row=row, column=8, sticky="w")

# Connect / Switch button

def _switch_cf():
    global URI, RIGID_BODY_NAME, RIGID_BODY_ID
    URI = cf_uri_var.get().strip()
    name = rb_name_var.get().strip()
    idtxt = rb_id_var.get().strip()
    RIGID_BODY_NAME = name if name else None
    RIGID_BODY_ID = int(idtxt) if idtxt.isdigit() else None
    # Restart link
    stop_cf_link()
    start_cf_link(URI)
    status_var.set(f"Status: Switching to {URI} (RB name={RIGID_BODY_NAME}, id={RIGID_BODY_ID})")

switch_btn = ttk.Button(main, text="Connect / Switch", command=_switch_cf)
switch_btn.grid(row=row, column=9, padx=(8, 0), sticky="w")

# Live state
row += 1
x_var = tk.StringVar(value="X: +0.000 m")
y_var = tk.StringVar(value="Y: +0.000 m")
z_var = tk.StringVar(value="Z: +0.000 m")
d_var = tk.StringVar(value="XY dist from takeoff: 0.000 m")
status_var = tk.StringVar(value="Status: Connecting…")
tgt_var = tk.StringVar(value="Target: (0.00, 0.00, 0.50) m | |e|=0.000 m")
err_var = tk.StringVar(value="Err: dx=+0.000, dy=+0.000, dz=+0.000 m  | |e|=0.000")
last_opti_var = tk.StringVar(value="—")
opti_var = tk.StringVar(value="OptiTrack: waiting for frames…")

for var in (x_var, y_var, z_var, d_var, tgt_var, err_var):
    ttk.Label(main, textvariable=var, font=("Consolas", 11)).grid(row=row, column=0, columnspan=10, sticky="w")
    row += 1
# Add a tiny OptiTrack status line
ttk.Label(main, textvariable=opti_var, font=("Consolas", 10)).grid(row=row, column=0, columnspan=10, sticky="w")
row += 1
# Add a tiny OptiTrack status line (so you can verify streaming before takeoff)
ttk.Label(main, textvariable=opti_var, font=("Consolas", 10)).grid(row=row, column=0, columnspan=6, sticky="w")
row += 1

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

# Start workers
# Start OptiTrack streaming immediately
threading.Thread(target=optitrack_worker, daemon=True).start()
threading.Thread(target=extpos_sender_worker, daemon=True).start()
# Start initial CF link using defaults; user can switch in GUI
start_cf_link(URI)

# GUI loop
root.after(200, update_gui)
root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
