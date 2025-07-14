import matplotlib.pyplot as plt
import math
import pandas as pd
import numpy as np

impacts = pd.read_csv("goodImpacts.csv")

# plt.hist(impacts[" Bully Velocity"], bins=200, rwidth=1)
# plt.title("Bully Velocity Histogram")
# plt.xlabel("Bully Velocity")
# plt.show()

impacts[" Bully Angle"] = impacts[" Bully Angle"].astype(float) / 65535 * 2 * math.pi
good_impacts = impacts[impacts[" Frames of Stability"] > 600]

print(good_impacts)

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ax.scatter(good_impacts[" Bully Angle"], good_impacts[" Bully Velocity"], s=2)
ax.set_xticklabels(['16384', '24576', '32768', '40960', '49152', '57344', '0', '8192'])
plt.grid(False)
ax.set_title("Bully Velocity/Angle Polar Plot", va='bottom')
plt.show()