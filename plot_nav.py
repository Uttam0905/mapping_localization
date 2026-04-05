import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("/home/uttam/nav_data.csv")

plt.figure()
plt.plot(df["x"], df["y"])

plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Robot Navigation Trajectory")

plt.grid()
plt.axis("equal")

plt.savefig("trajectory.png")
plt.show()