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
    """Make some nice plots, focusing on a particular range of steps."""
    data = pd.read_csv(csv_file)

    # Extract the relevant quantities
    solver_steps = data.index[start_step:end_step]
    k = data["k"][start_step:end_step]
    times = data["t"][start_step:end_step]
    time_steps = data["h"][start_step:end_step]
    residual = data["residual"][start_step:end_step]
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

    # Third plot is the momentum residual
    ax[2].plot(solver_steps, residual, "o")
    ax[2].set_ylabel("$\\nabla\ell$")
    ax[2].set_yscale("log")

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


def analyze_data(csv_file, start_step, end_step):
    """Answer some basic questions about the data."""
    data = pd.read_csv(csv_file)

    # Extract the relevant quantities
    solver_steps = np.array(data.index[start_step:end_step])
    k = np.array(data["k"][start_step:end_step])
    times = np.array(data["t"][start_step:end_step])
    time_steps = np.array(data["h"][start_step:end_step])
    residual = np.array(data["residual"][start_step:end_step])
    hessian_refresh = np.array(data["refresh_hessian"][start_step:end_step])
    problem_changed = np.array(data["problem_changed"][start_step:end_step])
    solve_phase = np.array(data["solve_phase"][start_step:end_step])

    print(f"Analyzing interval from t = {times[0]:.3f} to t = {times[-1]:.3f}")
    
    print("\nHow many time steps?")
    phase_0_solves = np.sum(np.logical_and(k == 0, solve_phase == 0))
    phase_1_solves = np.sum(np.logical_and(k == 0, solve_phase == 1))
    phase_2_solves = np.sum(np.logical_and(k == 0, solve_phase == 2))
    print("==> ", phase_0_solves, "full steps t --> t+h (phase 0)")
    print("==> ", phase_1_solves, "half steps t --> t+h/2 (phase 1)")
    print("==> ", phase_2_solves, "half steps t+h/2 --> t+h (phase 2)")
    
    print("\nHow many Newton steps in total?")
    num_newton_steps = end_step - start_step
    print("==> ", num_newton_steps)

    print("\nHow many Newton steps in each phase?")
    phase_0_steps = np.sum(solve_phase == 0)
    phase_1_steps = np.sum(solve_phase == 1)
    phase_2_steps = np.sum(solve_phase == 2)
    print("==> ", phase_0_steps, "(phase 0)")
    print("==> ", phase_1_steps, "(phase 1)")
    print("==> ", phase_2_steps, "(phase 2)")

    print("\nHow many Newton steps on average?")
    phase_0_avg = phase_0_steps / phase_0_solves
    phase_1_avg = phase_1_steps / phase_1_solves
    phase_2_avg = phase_2_steps / phase_2_solves
    print(f"==>  {phase_0_avg:.1f} (phase 0)")
    print(f"==>  {phase_1_avg:.1f} (phase 1)")
    print(f"==>  {phase_2_avg:.1f} (phase 2)")

    print("\nMaximum number of Newton steps in each phase?")
    phase_0_max = np.max(k[np.where(solve_phase == 0)])
    phase_1_max = np.max(k[np.where(solve_phase == 1)])
    phase_2_max = np.max(k[np.where(solve_phase == 2)])
    print("==> ", phase_0_max, "(phase 0)")
    print("==> ", phase_1_max, "(phase 1)")
    print("==> ", phase_2_max, "(phase 2)")

    print("\nHow many Hessian refreshes?")
    print("==> ", np.sum(hessian_refresh))

    print("\nHow many of those were forced by problem structure change?")
    print("==> ", np.sum(problem_changed))

    print("\nHow many Hessian refreshes occured in each phase?")
    phase_0_refreshes = np.sum(np.logical_and(hessian_refresh, solve_phase == 0))
    phase_1_refreshes = np.sum(np.logical_and(hessian_refresh, solve_phase == 1))
    phase_2_refreshes = np.sum(np.logical_and(hessian_refresh, solve_phase == 2))
    print("==> ", phase_0_refreshes, "(phase 0)")
    print("==> ", phase_1_refreshes, "(phase 1)")
    print("==> ", phase_2_refreshes, "(phase 2)")

    print("\nHow many problem structure changes in each phase?")
    phase_0_changes = np.sum(np.logical_and(problem_changed, solve_phase == 0))
    phase_1_changes = np.sum(np.logical_and(problem_changed, solve_phase == 1))
    phase_2_changes = np.sum(np.logical_and(problem_changed, solve_phase == 2))
    print("==> ", phase_0_changes, "(phase 0)")
    print("==> ", phase_1_changes, "(phase 1)")
    print("==> ", phase_2_changes, "(phase 2)")


if __name__ == "__main__":
    # Path to the csv file
    csv_file = "clutter_test.csv"

    # Set the data range (in terms of solver steps)
    start_step = 200
    end_step = 1000

    # Answer some questions about the data
    analyze_data(csv_file, start_step, end_step)

    # Make some plots
    make_plots(csv_file, 200, 1000)
