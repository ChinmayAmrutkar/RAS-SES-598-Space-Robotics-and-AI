import pandas as pd
import matplotlib.pyplot as plt

# Define case file names and labels
cases = {
    "Case 1": "case1.csv",
    "Case 2": "case2.csv",
    "Case 3": "case3.csv",
    "Case 4": "case4.csv",
    "Case 5": "case5.csv"
}

# Load data for all cases
data = {}
for case, filename in cases.items():
    data[case] = pd.read_csv(filename)

# Extract key metrics
metrics = ['Cart Position', 'Pole Angle', 'Control Force', 'Cart Velocity', 'Pole Angular Velocity']
titles = ['Cart Position (m)', 'Pole Angle (rad)', 'Control Force (N)', 'Cart Velocity (m/s)', 'Pole Angular Velocity (rad/s)']
colors = ['blue', 'red', 'green', 'purple', 'orange']

# Create subplots for each metric
fig, axes = plt.subplots(nrows=3, ncols=2, figsize=(12, 12))
axes = axes.flatten()

for i, metric in enumerate(metrics):
    for (case, color) in zip(cases.keys(), colors):
        axes[i].plot(data[case]['Time'], data[case][metric], label=case, color=color)
    
    axes[i].set_title(titles[i])
    axes[i].set_xlabel('Time (s)')
    axes[i].set_ylabel(metric)
    axes[i].legend()
    axes[i].grid()

# Hide the last empty subplot if needed
if len(metrics) % 2 != 0:
    fig.delaxes(axes[-1])

plt.tight_layout()
plt.suptitle('Comparison of All Cases', fontsize=14)
plt.subplots_adjust(top=0.92)
plt.show()
