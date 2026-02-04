# 2 crazyflies flying in square parallel

# Two-CF simultaneous square using threads (no Swarm wrapper).
# - Sequential link open (robust)
# - Per-CF prep: HL on, Kalman, Position controller, EKF reset
# - Takeoff together
# - Square flown in parallel threads (with per-CF offsets)
# - Land together
# - No cache (rw_cache=None)

import time
import threading
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URIS = [
    "radio://0/80/2M/E7E7E7E701",  # CF1
    "radio://0/80/2M/E7E7E7E702",  # CF2
]

# ---- Flight tuning ----
TAKEOFF_Z = 0.50      # m
TAKEOFF_T = 2.0       # s
HOVER_T   = 0.8       # s pause between edges
LAND_T    = 2.0       # s

BOX_SIZE  = 0.45      # m (reduce to 0.30 if drift is large)
EDGE_T    = 2.0       # s per edge
YAW       = 0.0       # deg

# Keep the drones separated
OFFSETS = {
    "radio://0/80/2M/E7E7E7E701": (-0.30,  0.00),
    "radio://0/80/2M/E7E7E7E702": ( 0.30,  0.00),
}

def setp(cf, name, value):
    try:
        cf.param.set_value(name, str(value))
        return True
    except Exception as e:
        print(f"  set {name}={value} failed: {e}")
        return False

def prep_cf(cf):
    # Enable HL + Kalman + Position controller
    setp(cf, "commander.enHighLevel", 1)
    setp(cf, "stabilizer.estimator", 2)   # 2 = Kalman
    setp(cf, "stabilizer.controller", 2)  # 2 = Mellinger/Position
    # Reset EKF and settle
    setp(cf, "kalman.resetEstimation", 1); time.sleep(0.12)
    setp(cf, "kalman.resetEstimation", 0); time.sleep(1.2)

def rel_go(hl, dx, dy, dz, t, yaw=0.0, pause=0.0):
    hl.go_to(dx, dy, dz, yaw, t, relative=True)
    time.sleep(t + pause)

def square_sequence(hl, x0=0.0, y0=0.0):
    # Shift to start offset at current altitude
    rel_go(hl, x0, y0, 0.0, t=1.2, pause=0.2)
    # 4 edges
    rel_go(hl,  BOX_SIZE,  0.0, 0.0, EDGE_T, YAW, HOVER_T)
    rel_go(hl,  0.0,       BOX_SIZE, 0.0, EDGE_T, YAW, HOVER_T)
    rel_go(hl, -BOX_SIZE,  0.0, 0.0, EDGE_T, YAW, HOVER_T)
    rel_go(hl,  0.0,      -BOX_SIZE, 0.0, EDGE_T, YAW, HOVER_T)

def worker_square(uri, scf):
    hl = scf.cf.high_level_commander
    x0, y0 = OFFSETS.get(uri, (0.0, 0.0))
    square_sequence(hl, x0=x0, y0=y0)

def main():
    cflib.crtp.init_drivers()

    scfs = []
    try:
        # 1) open both links sequentially (no cache)
        for uri in URIS:
            for k in range(1, 6):
                try:
                    scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache=None))
                    scf.open_link()
                    print(f"[OK] Opened {uri} (attempt {k})")
                    scfs.append(scf)
                    break
                except Exception as e:
                    print(f"[{k}/5] Failed to open {uri}: {e}")
                    time.sleep(0.8 + 0.2*k)
            else:
                raise SystemExit(f"Could not open {uri}")

        # 2) prep both
        for i, scf in enumerate(scfs):
            print(f"Prep CF{i+1}...")
            prep_cf(scf.cf)

        time.sleep(0.4)

        # 3) takeoff together
        print("Takeoff both...")
        for scf in scfs:
            scf.cf.high_level_commander.takeoff(TAKEOFF_Z, TAKEOFF_T)
        time.sleep(TAKEOFF_T + 0.5)

        # 4) run squares in parallel threads
        print("Square together...")
        threads = []
        for i, scf in enumerate(scfs):
            t = threading.Thread(target=worker_square, args=(URIS[i], scf), daemon=True)
            t.start()
            threads.append(t)
        for t in threads:
            t.join()

        # 5) land together
        print("Land both...")
        for scf in scfs:
            scf.cf.high_level_commander.land(0.0, LAND_T)
        time.sleep(LAND_T + 0.6)

        # 6) stop commanders
        for scf in scfs:
            scf.cf.high_level_commander.stop()

        print("Done.")

    finally:
        for scf in scfs:
            try: scf.close_link()
            except: pass

if __name__ == "__main__":
    main()
