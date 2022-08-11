#!/usr/bin/env python

import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

import numpy as np
import os
import sys

##
#
# A quick script to make a plot of solution data for the
# acrobot swingup problem. 
#
# This script must be run from the "drake/" directory. 
#
##

# drake/ directory, contains drake/bazel-out symlink
drake_root = os.getcwd()

# Define our optimization problem
dt = 5e-2
num_steps = 40
max_iters = 250
gravity = 9.81

Qq = 0.0
Qv = 1e-1
R = 1e-1
unactuated_penalty = 1e3
Qfq = 1e2
Qfv = 1e0

# Solve the optimization problem
options_string = " -- "
options_string += "--visualize=false "
options_string += "--save_data=true "
options_string += f"--time_step={dt} "
options_string += f"--num_steps={num_steps} "
options_string += f"--max_iters={max_iters} "
options_string += f"--Qq={Qq} "
options_string += f"--Qv={Qv} "
options_string += f"--R={R} "
options_string += f"--unactuated_penalty={unactuated_penalty} "
options_string += f"--Qfq={Qfq} "
options_string += f"--Qfv={Qfv} "
options_string += f"--gravity={gravity} "
options_string += f"--linesearch=armijo "

os.system("cd " + drake_root)
code = os.system("bazel run //traj_opt/examples:acrobot" + options_string)

if code != 0:
    # Solving failed (probably a compiler error)
    sys.exit()

# Bazel stores files in strange places
data_file = drake_root + "/bazel-out/k8-opt/bin/traj_opt/examples/acrobot.runfiles/drake/acrobot_data.csv"

# Read data from the file and format nicely
data = np.genfromtxt(data_file, delimiter=',', names=True)
iters = data["iter"]

# Make plots
f, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5,1,sharex=True,figsize=(8,11))
ax1.set_title(f"penalty={unactuated_penalty}, dt={dt}, N={num_steps}, Qq={Qq}, Qv={Qv}, R={R}, Qfq={Qfq}, Qfv={Qfv}")

ax1.plot(iters, data["time"])
ax1.set_ylabel("Iteration time (s)")
ax1.set_ylim((0,0.01))

ax2.plot(iters, data["cost"] - data["cost"][-1])
ax2.set_ylabel("Cost (minus baseline)")
ax2.set_yscale("log")

ax3.plot(iters, data["ls_iters"])
ax3.set_ylabel("Linesearch Iters")

ax4.plot(iters, data["alpha"])
ax4.set_ylabel("alpha")

ax5.plot(iters, data["grad_norm"] / data["cost"])
ax5.set_ylabel("$||g||$ / cost")
ax5.set_yscale("log")
ax5.set_yticks(np.logspace(-12,0,7))

ax5.set_xlabel("Iteration")
ax5.xaxis.set_major_locator(MaxNLocator(integer=True))

plt.show()
