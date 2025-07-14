import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("RibbonResults.csv")

chunk_of_interest = df[(df["Start Velocity"] >= 276600) & (df["Start Velocity"] <= 277000)]

print(len(chunk_of_interest))

# sorted_chunk = chunk_of_interest.sort_values("End Velocity")

speedy_chunk_of_interest = chunk_of_interest[chunk_of_interest["End Velocity"] < -7e6]

print(len(speedy_chunk_of_interest))

plt.hist(speedy_chunk_of_interest["End Velocity"], bins=30)
plt.show()

df2 = pd.read_csv("HighSpeedSticks.csv")

print(len(df2))

x = df2["x"]
y = df2["y"]

# Create the scatter plot
plt.scatter(x, y)

plt.xlabel('Stick X')
plt.ylabel('Stick Y')

plt.show()
