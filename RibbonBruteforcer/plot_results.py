import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv("RibbonResults.csv")

dist = 129634

# nYs = (4 * dist) / df["Start Velocity"] - 1

fig = plt.figure(figsize=(25, 10))

# Set the face color of the axis object to black
ax = fig.add_subplot(111, facecolor='black')

scatter = ax.scatter(x=df["Start Velocity"], y=df["Cam Yaw"], c=df["End Velocity"], s=1, cmap='jet_r')
# scatter = ax.scatter(x=nYs, y=df["Cam Yaw"], c=df["End Velocity"], s=1, cmap='Greys', vmin=-6.5e6, vmax=-6.5e6+0.0001)

ax.set_xlabel("Start Velocity")
# ax.set_xlabel("nYs")
ax.set_ylabel("Cam Yaw")

cbar = plt.colorbar(scatter)
cbar.ax.invert_yaxis()

plt.show()
