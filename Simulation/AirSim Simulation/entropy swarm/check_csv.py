import pandas as pd
import numpy as np

df = pd.read_csv(
    r"C:\Users\taram\source\repos\SwarmGUI_NEW\AirSim\EntropyRewrite\data-files\entropySave.csv",
    sep=r"\s+",
    engine="python"
)

print("Num rows:", len(df))
print("Columns:", df.columns.tolist())

t = df["TimeDifference"].to_numpy(float)
print("First time:", t[0])
print("Last time :", t[-1])
print("Total time (s):", t[-1] - t[0])

x1 = df["X1"].to_numpy(float)
y1 = df["Y1"].to_numpy(float)
dx = np.diff(x1)
dy = np.diff(y1)
steps = np.sqrt(dx**2 + dy**2)
print("Drone1 path length from CSV:", steps.sum())
print("Max single step:", steps.max())
