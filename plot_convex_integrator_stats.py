#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Relative path to the data file (change as needed)
data_file = "bazel-bin/examples/multibody/clutter/clutter.runfiles/_main/convex_integrator_stats.csv"

# Load the data and extract the relevant columns
data = pd.read_csv(data_file)
times = data['time'].values
iterations = data['iteration'].values
ls_iterations = data['ls_iterations'].values

# Compute the total number of linesearch and solver iterations for each timestep
timesteps = []
total_iterations = []
total_ls_iterations = []
avg_ls_iterations = []

for i in range(len(times)):
    if i == 0 or times[i] != times[i - 1]:
        timesteps.append(times[i])
        total_iterations.append(iterations[i] + 1)
        total_ls_iterations.append(ls_iterations[i])
    else:
        # Iteration count is just the highest number
        total_iterations[-1] = iterations[i] + 1

        # Linesearch iterations are cumulative
        total_ls_iterations[-1] += ls_iterations[i]

for i in range(len(total_ls_iterations)):
    avg_ls_iterations.append(total_ls_iterations[i] / total_iterations[i])

# Print some overall stats
print("Total Timesteps             :", len(timesteps))
print("Total Solver Iterations     :", sum(total_iterations))
print("Total Linesearch Iterations :", sum(total_ls_iterations))

# Make plots
plt.rcParams.update({'font.size': 14})
fig, ax = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

ax[0].plot(timesteps, total_iterations, marker='o', label='Total Iterations')
ax[0].set_ylabel('Solver Iterations')
ax[0].grid()

ax[1].plot(timesteps, avg_ls_iterations, marker='x', label='Average Linesearch Iterations', color='green')
ax[1].set_ylabel('Linesearch Iterations\n(avg per solver iteration)')
ax[1].set_xlabel('Simulation Time (s)')
ax[1].grid()

plt.tight_layout()
plt.show()


