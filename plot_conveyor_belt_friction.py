#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

##
#
# Run the conveyor belt example first:
#  bazel run //examples/multibody/inclined_plane_with_body:conveyor_belt
#
##

# Relative path to the data
data_file = "bazel-bin/examples/multibody/inclined_plane_with_body/conveyor_belt.runfiles/_main/conveyor_belt_data.csv"

# Load the data and extract the relevant columns
data = pd.read_csv(data_file)
times = data['time'].values
ft = data['ft'].values
vt = data['vt'].values

# Make plots
plt.rcParams.update({'font.size': 14})
fig, ax = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

ax[0].plot(times, vt)
ax[0].set_ylabel('Velocity $v_t$')
ax[0].grid()

ax[1].plot(times, ft)
ax[1].set_ylabel('Friction $f_t$')
ax[1].grid()

plt.tight_layout()
plt.show()
