#!/usr/bin/env python

##
#
# Make a plot of the cost landscape for a super simple system with two DoFs and
# one timestep (2 total decision variables).
#
##

import matplotlib.pyplot as plt
from matplotlib import ticker
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os
import sys

# Determine what exactly to plot based on command-line arguments
if len(sys.argv) == 1:
    # If no iteration number is specified, we'll use the first iteration
    i = 0
elif len(sys.argv) == 2:
    i = int(sys.argv[1])
else:
    print(f"Usage: {sys.argv[0]} [iteration = 0]")
    sys.exit(1)

# Get data for the contour plot
# N.B. Must run this script from the drake/ directory (containing bazel-out symlink)
drake_root = os.getcwd()
data_file = drake_root + "/bazel-out/k8-opt/bin/traj_opt/examples/2dof_spinner.runfiles/drake/contour_data.csv"
ns = 150    # number of sample points in each axis

data = np.genfromtxt(data_file, delimiter=',', names=True)
q1 = data["q1"].reshape((ns,ns))
q2 = data["q2"].reshape((ns,ns))
cost = data["L"].reshape((ns,ns))
g1 = data["g1"].reshape((ns,ns))
g2 = data["g2"].reshape((ns,ns))
H11 = data["H11"].reshape((ns,ns))
H12 = data["H12"].reshape((ns,ns))
H21 = data["H21"].reshape((ns,ns))
H22 = data["H22"].reshape((ns,ns))

# Plot cost as a function of q
plt.figure()
levels = np.linspace(np.min(cost), np.max(cost), 50)
plt.contour(q1, q2, cost, levels=levels)
plt.title("Cost")
plt.xlabel("$q_1$")
plt.ylabel("$q_2$")

# Plot the gradient in each direction as a function of q
plt.figure()
plt.suptitle("Gradient")

plt.subplot(2,1,1)
levels = np.linspace(np.min(g1), np.max(g1), 50)
plt.contour(q1, q2, g1, levels=levels)
plt.title("$g_1$")
plt.xlabel("$q_1$")
plt.ylabel("$q_2$")

plt.subplot(2,1,2)
levels = np.linspace(np.min(g2), np.max(g2), 50)
plt.contour(q1, q2, g2, levels=levels)
plt.title("$g_2$")
plt.xlabel("$q_1$")
plt.ylabel("$q_2$")

# Plot each element of the Hessian as a function of q

plt.figure()
plt.suptitle("Hessian")

plt.subplot(2,2,1)
levels = np.linspace(np.min(H11), np.max(H11), 50)
plt.contour(q1, q2, H11, levels=levels)
plt.title("$H_{11}$")
plt.xlabel("$q_1$")
plt.ylabel("$q_2$")

plt.subplot(2,2,2)
levels = np.linspace(np.min(H12), np.max(H12), 50)
plt.contour(q1, q2, H12, levels=levels)
plt.title("$H_{12}$")
plt.xlabel("$q_1$")
plt.ylabel("$q_2$")

plt.subplot(2,2,3)
levels = np.linspace(np.min(H21), np.max(H21), 50)
plt.contour(q1, q2, H21, levels=levels)
plt.title("$H_{21}$")
plt.xlabel("$q_1$")
plt.ylabel("$q_2$")

plt.subplot(2,2,4)
levels = np.linspace(np.min(H22), np.max(H22), 50)
plt.contour(q1, q2, H22, levels=levels)
plt.title("$H_{22}$")
plt.xlabel("$q_1$")
plt.ylabel("$q_2$")

plt.show()
