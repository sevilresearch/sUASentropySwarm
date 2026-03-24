import pandas as pd
import numpy as np
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # no GUI; saves PNGs only
import matplotlib.pyplot as plt


# =========================
# PATHS
# =========================
SCRIPT_DIR = Path(__file__).resolve().parent
CSV_PATH = SCRIPT_DIR / "data-files" / "entropySave.csv"

print("CSV_PATH =", CSV_PATH)
if not CSV_PATH.exists():
    raise FileNotFoundError(f"CSV not found: {CSV_PATH}")

OUT_DIR = SCRIPT_DIR / "plots"
OUT_DIR.mkdir(parents=True, exist_ok=True)


# =========================
# LOAD CSV
# =========================
df = pd.read_csv(CSV_PATH, sep=r"\s+", engine="python")
print("Num rows:", len(df))

print("TimeDifference head:", df["TimeDifference"].head(10).to_list())
print("TimeDifference tail:", df["TimeDifference"].tail(3).to_list())

# =========================
# INFER NUMBER OF DRONES
# =========================
cols = df.columns.tolist()
ids = []
for c in cols:
    if str(c).startswith("X"):
        try:
            ids.append(int(str(c)[1:]))
        except:
            pass

N = max(ids) if ids else 0
if N == 0:
    raise RuntimeError("No X1/X2/... columns found.")

def col_np(name: str) -> np.ndarray:
    """Get numeric numpy array from a dataframe column."""
    return pd.to_numeric(df[name], errors="coerce").to_numpy()

def path_length_xy(x: np.ndarray, y: np.ndarray):
    """Return (total_distance, avg_step_distance, num_steps) using XY arrays."""
    m = ~(np.isnan(x) | np.isnan(y))
    x, y = x[m], y[m]
    if len(x) < 2:
        return 0.0, 0.0, 0
    steps = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    return float(np.nansum(steps)), float(np.nanmean(steps)), int(len(steps))


# =========================
# DISTANCE PER DRONE
# =========================
dist_per_drone = []
avgstep_per_drone = []

for i in range(1, N + 1):
    x = col_np(f"X{i}")
    y = col_np(f"Y{i}")
    total_d, avg_step, _ = path_length_xy(x, y)
    dist_per_drone.append(total_d)
    avgstep_per_drone.append(avg_step)

avg_distance = float(np.mean(dist_per_drone)) if dist_per_drone else 0.0
print("Per-drone distances (m):", [round(d, 2) for d in dist_per_drone])
print("Average distance (m):", round(avg_distance, 2))


# =========================
# TIME AXIS (FIXED)
# =========================
if "TimeDifference" in df.columns:
    td = col_np("TimeDifference")

    # Detect whether TimeDifference is:
    #  - elapsed time (monotonically increasing), OR
    #  - per-sample dt (small values that need cumsum)
    finite = np.isfinite(td)
    if finite.sum() >= 3:
        td_f = td[finite]
        d = np.diff(td_f)

        looks_monotonic = np.all(d >= -1e-3)          # allow tiny floating error
        looks_like_elapsed = looks_monotonic and (td_f[-1] > 1.0)

        if looks_like_elapsed:
            t = td
            t_label = "Time (s)"
        else:
            t = np.cumsum(np.nan_to_num(td, nan=0.0))
            t_label = "Time (s)"
    else:
        t = td
        t_label = "Time (s)"

elif "Time" in df.columns:
    t = col_np("Time")
    t_label = "Time (s)"
else:
    t = np.arange(len(df), dtype=float)
    t_label = "Index"


# =========================
# PLOT 1: XY TRAJECTORIES
# =========================
plt.figure(figsize=(7, 6))
for i in range(1, N + 1):
    x = col_np(f"X{i}")
    y = col_np(f"Y{i}")
    total_d, avg_step, _ = path_length_xy(x, y)

    m = ~(np.isnan(x) | np.isnan(y))
    plt.plot(x[m], y[m], label=f"CF{i} (distance={total_d:.2f}m)") #, avgStep={avg_step:.2f}

plt.xlabel("X(m)")
plt.ylabel("Y(m)")
plt.title("XY Trajectories")
plt.axis("equal")
plt.legend()
plt.savefig(OUT_DIR / "xy_trajectories.png", dpi=200, bbox_inches="tight")
plt.close()


# =========================
# PLOT 2: ENTROPY VS TIME
# =========================
entropy_cols = []
for i in range(1, N + 1):
    for cand in [f"EntropyCF{i}", f"Entropy{i}", f"St{i}", f"S{i}"]:
        if cand in df.columns:
            entropy_cols.append(cand)
            break

if entropy_cols:
    plt.figure(figsize=(8, 4))
    for c in entropy_cols:
        y = col_np(c)  # numpy array
        m = ~(np.isnan(t) | np.isnan(y))
        plt.plot(t[m], y[m], label=c)

    plt.xlabel(t_label)
    plt.ylabel("Entropy")
    plt.title("Entropy vs Time")
    plt.legend()
    plt.savefig(OUT_DIR / "entropy_vs_time.png", dpi=200, bbox_inches="tight")
    plt.close()
else:
    print("No entropy columns found; skipping entropy plot.")


# =========================
# PLOT 3: INTER-DRONE DISTANCES
# =========================
def pair_dist(i, j):
    xi = col_np(f"X{i}")
    yi = col_np(f"Y{i}")
    xj = col_np(f"X{j}")
    yj = col_np(f"Y{j}")
    return np.sqrt((xi - xj)**2 + (yi - yj)**2)

plt.figure(figsize=(8, 4))
if N >= 3:
    d12 = pair_dist(1, 2)
    d23 = pair_dist(2, 3)
    d13 = pair_dist(1, 3)

    m = ~(np.isnan(t) | np.isnan(d12))
    plt.plot(t[m], d12[m], label="CF1-2")
    m = ~(np.isnan(t) | np.isnan(d23))
    plt.plot(t[m], d23[m], label="CF2-3")
    m = ~(np.isnan(t) | np.isnan(d13))
    plt.plot(t[m], d13[m], label="CF1-3")
else:
    for i in range(1, N + 1):
        for j in range(i + 1, N + 1):
            dij = pair_dist(i, j)
            m = ~(np.isnan(t) | np.isnan(dij))
            plt.plot(t[m], dij[m], label=f"d{i}{j}")

plt.xlabel(t_label)
plt.ylabel("Distance (m)")
plt.title("Inter-CF Distance vs Time")
plt.legend()
plt.savefig(OUT_DIR / "distances_vs_time.png", dpi=200, bbox_inches="tight")
plt.close()

print("\nSaved plots to:", OUT_DIR)