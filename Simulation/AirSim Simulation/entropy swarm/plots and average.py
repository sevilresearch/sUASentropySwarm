'''
Created by Juan José Molina - March 2026
'''

import pandas as pd
import numpy as np
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # IMPORTANT: prevents Qt/GUI hang
import matplotlib.pyplot as plt


CSV_PATH = Path(r"C:\Users\juanj\AirSim\PythonClient\EntropyTARA\data-files\entropySave.csv")
OUT_DIR = CSV_PATH.parent / "plots"
OUT_DIR.mkdir(parents=True, exist_ok=True)

df = pd.read_csv(CSV_PATH, sep=r"\s+", engine="python")
print("Num rows:", len(df))

# Infer N from X1, X2, ...
cols = df.columns.tolist()
ids = []
for c in cols:
    if c.startswith("X"):
        try:
            ids.append(int(c[1:]))
        except:
            pass
N = max(ids) if ids else 0
if N == 0:
    raise RuntimeError("No X1/X2/... columns found.")

def path_length(x, y):
    x = pd.to_numeric(x, errors="coerce").to_numpy()
    y = pd.to_numeric(y, errors="coerce").to_numpy()
    m = ~(np.isnan(x) | np.isnan(y))
    x, y = x[m], y[m]
    if len(x) < 2:
        return 0.0, 0.0, 0
    steps = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    return float(np.sum(steps)), float(np.mean(steps)), int(len(steps))

# Distance per drone
dist_per_drone = []
avgstep_per_drone = []
for i in range(1, N + 1):
    total_d, avg_step, _ = path_length(df[f"X{i}"], df[f"Y{i}"])
    dist_per_drone.append(total_d)
    avgstep_per_drone.append(avg_step)

avg_distance = float(np.mean(dist_per_drone))
print("Per-drone distances (m):", [round(d, 2) for d in dist_per_drone])
print("Average distance (m):", round(avg_distance, 2))

# Time axis
if "TimeDifference" in df.columns:
    dt = pd.to_numeric(df["TimeDifference"], errors="coerce").to_numpy()
    t = np.cumsum(np.nan_to_num(dt, nan=0.0))
    t_label = "Elapsed time (sum of TimeDifference)"
elif "Time" in df.columns:
    t = pd.to_numeric(df["Time"], errors="coerce").to_numpy()
    t_label = "Time"
else:
    t = np.arange(len(df), dtype=float)
    t_label = "Index"

# -------- Plot 1: XY trajectories --------
plt.figure(figsize=(7, 6))
for i in range(1, N + 1):
    total_d, avg_step, _ = path_length(df[f"X{i}"], df[f"Y{i}"])
    plt.plot(df[f"X{i}"], df[f"Y{i}"], label=f"Drone{i} (total={total_d:.2f}, avgStep={avg_step:.2f})")
plt.xlabel("X")
plt.ylabel("Y")
plt.title("XY Trajectories")
plt.axis("equal")
plt.legend()
plt.savefig(OUT_DIR / "xy_trajectories.png", dpi=200, bbox_inches="tight")
plt.close()

# -------- Plot 2: Entropy vs time --------
entropy_cols = []
for i in range(1, N + 1):
    for cand in [f"EntropyDrone{i}", f"Entropy{i}", f"St{i}", f"S{i}"]:
        if cand in df.columns:
            entropy_cols.append((i, cand))
            break

if entropy_cols:
    plt.figure(figsize=(8, 4))
    for i, c in entropy_cols:
        plt.plot(t, pd.to_numeric(df[c], errors="coerce"), label=c)
    plt.xlabel(t_label)
    plt.ylabel("Entropy")
    plt.title("Entropy vs Time")
    plt.legend()
    plt.savefig(OUT_DIR / "entropy_vs_time.png", dpi=200, bbox_inches="tight")
    plt.close()
else:
    print("No entropy columns found; skipping entropy plot.")

# -------- Plot 3: Inter-drone distances --------
def pair_dist(i, j):
    xi = pd.to_numeric(df[f"X{i}"], errors="coerce").to_numpy()
    yi = pd.to_numeric(df[f"Y{i}"], errors="coerce").to_numpy()
    xj = pd.to_numeric(df[f"X{j}"], errors="coerce").to_numpy()
    yj = pd.to_numeric(df[f"Y{j}"], errors="coerce").to_numpy()
    return np.sqrt((xi - xj)**2 + (yi - yj)**2)

plt.figure(figsize=(8, 4))
if N >= 3:
    plt.plot(t, pair_dist(1, 2), label="d12")
    plt.plot(t, pair_dist(2, 3), label="d23")
    plt.plot(t, pair_dist(1, 3), label="d13")
else:
    for i in range(1, N + 1):
        for j in range(i + 1, N + 1):
            plt.plot(t, pair_dist(i, j), label=f"d{i}{j}")

plt.xlabel(t_label)
plt.ylabel("Distance")
plt.title("Inter-drone Distance vs Time")
plt.legend()
plt.savefig(OUT_DIR / "distances_vs_time.png", dpi=200, bbox_inches="tight")
plt.close()

print("\nSaved plots to:", OUT_DIR)