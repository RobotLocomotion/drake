#!/usr/bin/env python

##
#
# Make plots of cost, gradient, and Hessian as a function of q for a simple
# system with one DoF and one timestep (and therefore only 1 decision variable).
#
##

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

# Choose iteration to highlight
if len(sys.argv) == 2:
    iter = int(sys.argv[1])
else:
    iter = 0

# Load lineplot data from a file
drake_root = os.getcwd()
data_file = drake_root + "/bazel-out/k8-opt/bin/traj_opt/examples/wall_ball.runfiles/drake/lineplot_data.csv"

data = np.genfromtxt(data_file, delimiter=',', names=True)
q = data["q"]
L = data["L"]
g = data["g"]
H = data["H"]

# Load iteration data from a file
iteration_data_file = drake_root + "/bazel-out/k8-opt/bin/traj_opt/examples/wall_ball.runfiles/drake/iteration_data.csv"
iteration_data = np.genfromtxt(iteration_data_file, delimiter=',', names=True)
q_iter = iteration_data["q"]
L_iter = iteration_data["cost"]
delta = iteration_data["Delta"]
delta_q = iteration_data["dq"]
rho = iteration_data["rho"]

print(f"iter : {iter}")
print(f"q    : {q_iter[iter]}")
print(f"Δ    : {delta[iter]}")
print(f"Δq   : {delta_q[iter]}")
print(f"ρ(Δq): {rho[iter]}")
print("")

# Make a plot of trust ratio as a function of dq around a particular q
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(8,11))

q0 = q_iter[iter]    # nominal value that we want to center around
idx = np.argmin(abs(q-q0))    # index of value closest to q0

q_center = q[idx]
dq = q - q[idx]

actual_reduction = L[idx] - L
model_reduction = []
for i in range(len(q)):
    dq_i = q[i] - q[idx]
    g_i = g[idx]
    H_i = H[idx]
    L_expected = L[idx] + g_i*dq_i + 0.5 * dq_i * H_i * dq_i
    model_reduction.append(L[idx] - L_expected)
model_reduction = np.asarray(model_reduction)
trust_ratio = actual_reduction / model_reduction

ax1.plot(dq, trust_ratio)
ax1.set_title(f"trust ratio around q={q_center}")
ax1.set_xlabel("Δq")
ax1.set_ylabel("ρ")
ax1.set_ylim((-1,3))
ax1.axhline(0.0, color='grey', linestyle="--")
ax1.axhline(1.0, color='grey', linestyle="--")
ax1.xaxis.set_tick_params(labelbottom=True)

# Vertical lines show q at this iteration, 
# trust region Δ, and candidate change Δq
ax1.axvline(0.0, color='red', linestyle='--')
ax1.axvspan(-delta[iter], delta[iter], color='grey', alpha=0.2)
ax1.axvline(delta_q[iter], color='blue', linestyle='--')
ax1.set_xlim((np.min(dq), np.max(dq)))

ax2.plot(dq, -actual_reduction, label="actual cost")
ax2.plot(dq, -model_reduction, label="model cost")
ax2.set_xlabel("Δq")
ax2.set_ylabel("L(q + Δq) - L(q)")
ax2.legend()

# Make plots of cost, gradient and Hessian
fig, (ax1, ax2, ax3) = plt.subplots(3,1, sharex=True, figsize=(8,11))

ax1.plot(q,L-np.min(L))
ax1.set_xlabel("q")
ax1.set_ylabel("Cost (minus baseline)")
ax1.xaxis.set_tick_params(labelbottom=True)

# Overlay values of (q,L) at each iteration
ax1.plot(q_iter, L_iter - np.min(L), 'rx')
#for i in range(len(q_iter)):
#    ax1.annotate(i, (q_iter[i], L_iter[i] - np.min(L)))

# Vertical lines show q at this iteration, 
# trust region Δ, and candidate change Δq
ax1.axvline(q_iter[iter], color='red', linestyle='--')
ax1.axvspan(q_iter[iter]-delta[iter], q_iter[iter]+delta[iter], color='grey', alpha=0.2)
ax1.axvline(q_iter[iter]+delta_q[iter], color='blue', linestyle='--')
ax1.set_xlim((np.min(q), np.max(q)))
ax1.set_ylim((0, np.max(L)-np.min(L)))

ax2.plot(q,g)
ax2.set_xlabel("q")
ax2.set_ylabel("Gradient")
ax2.axhline(0.0, color='grey', linestyle="--")
ax2.xaxis.set_tick_params(labelbottom=True)

ax3.plot(q,H)
ax3.set_xlabel("q")
ax3.set_ylabel("Hessian")

# Display all plots
plt.show()
