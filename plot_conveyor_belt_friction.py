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
vt = data["vt"].values  # tangential velocity
f_app = data["f_app"].values  # applied external force

# Compute friction force based on measured accelerations
dt = times[1] - times[0]
at = (vt[1:] - vt[0:-1]) / dt
at = np.insert(at, 0, 0.0)  # initial acceleration is zero
ft = at - f_app  # assumes mass = 1kg

# Make plots
plt.rcParams.update({'font.size': 14})
fig, ax = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

ax[0].plot(times, vt)
ax[0].set_ylabel('Velocity $v_t$')
ax[0].grid()

ax[1].plot(times, ft)
ax[1].plot(times, -f_app, linestyle="--")
ax[1].set_ylabel('Friction $f_t$')
ax[1].grid()

plt.tight_layout()
plt.show()
