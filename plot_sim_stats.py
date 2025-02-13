#!/usr/bin/env python

##
#
# Make a plot of simulation statistics for the convex integrator.
#
# Workflow:
#   - Run a sim and dump the stats to a file.
#      `bazel run examples/multibody/clutter -- --mbp_time_step=0 --simulator_integration_scheme=convex > clutter_test.csv`
#   - Clean up the csv file manually (remove lines at beginning and end)
#   - Run this script
#
##

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def make_plots(csv_file, start_step, end_step):
    """Make some nice plots, focusing on a particular range of steps"""
    data = pd.read_csv(csv_file)

    # Extract the relevant quantities
    solver_steps = data.index[start_step:end_step]
    k = data["k"][start_step:end_step]
    times = data["t"][start_step:end_step]
    time_steps = data["h"][start_step:end_step]
    costs = data["cost"][start_step:end_step]
    hessian_refresh = data["refresh_hessian"][start_step:end_step]
    problem_changed = data["problem_changed"][start_step:end_step]
    solve_phase = data["solve_phase"][start_step:end_step]

    # Set up the figure
    fig, ax = plt.subplots(5, 1, figsize=(10, 10), sharex=True)

    # First plot is the sim time
    ax[0].plot(solver_steps, times, "o")
    ax[0].set_ylabel("Time (s)")

    # Second plot is the time step
    ax[1].plot(solver_steps, time_steps, "o")
    ax[1].set_ylabel("$\delta t$ (s)")
    ax[1].set_yscale("log")

    # Third plot is the cost
    ax[2].plot(solver_steps, costs, "o")
    ax[2].set_ylabel("Cost $\ell$")

    # Fourth plot is a vertical bar when the hessian is refreshed. Uses axvline
    # because the refresh is not periodic
    for i, refresh, problem in zip(
        solver_steps, hessian_refresh, problem_changed
    ):
        if problem:
            ax[3].axvline(i, color="r", linewidth=3)
        elif refresh:
            ax[3].axvline(i, color="b", linewidth=3)
    ax[3].set_ylabel("Hessian refresh\n(red=forced)")

    # Fifth plot is the Newton step
    ax[4].plot(solver_steps, k, "o")
    ax[4].set_ylabel("Newton step $k$")

    # Set major ticks at the beginning of each time step
    major_ticks = (
        np.where(np.logical_and(solve_phase == 0, k == 0))[0] + start_step
    )
    ax[4].set_xticks(major_ticks)

    # Set minor ticks at the beginning of each solve phase (full step, first half
    # step, second half step)
    minor_ticks = np.where(k == 0)[0] + start_step
    ax[4].set_xticks(minor_ticks, minor=True)

    # Set vertical grid lines for both major and minor ticks
    for a in ax:
        a.grid(which="major", axis="x", color="k")
        a.grid(which="minor", axis="x")

    plt.xlabel("Solver Step")
    plt.show()


if __name__ == "__main__":
    # Path to the csv file
    csv_file = "clutter_test.csv"

    # Set the data range (in terms of solver steps)
    start_step = 200
    end_step = 1000

    # Make some plots
    make_plots(csv_file, 200, 1000)

    # Answer some questions about the data
