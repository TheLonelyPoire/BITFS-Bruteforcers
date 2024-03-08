import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv("RibbonResults.csv")

fig = plt.figure(figsize=(8, 6))

# Set the face color of the axis object to black
ax = fig.add_subplot(111, facecolor='black')

scatter = ax.scatter(x=df["Start Velocity"], y=df["Cam Yaw"], c=df["End Velocity"], s=1, cmap='jet_r')
ax.set_xlabel("Start Velocity")
ax.set_ylabel("Cam Yaw")
cbar = plt.colorbar(scatter)
cbar.ax.invert_yaxis()
plt.show()
