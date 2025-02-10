import argparse

import matplotlib.pyplot as plt
from pydrake.all import *

from error_control_demo import (
    run_simulation,
    gripper,
    ball_on_table,
    double_pendulum,
    cylinder_hydro,
    cylinder_point,
    clutter,
)

##
#
# Generate work-precision plots for various error-controlled integrators.
#
# Example usage: `bazel run work_precision_plots -- --example=double_pendulum
# --convex 0.1 0.01 0.001 0.0001 --runge_kutta3 0.1 0.01 0.001 0.0001
# --implicit_euler 0.1 0.01 0.001 0.0001`
#
##


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--example",
        type=str,
        required=True,
        help=(
            "Which example to run. One of: gripper, ball_on_table, "
            "double_pendulum, cylinder_hydro, cylinder_point, clutter. "
        ),
    )
    parser.add_argument(
        "--max_step_size",
        type=float,
        default=0.01,
        help=(
            "Maximum time step size (or fixed step size for discrete "
            "integrator). Default: 0.01."
        ),
    )

    # User supplies the integrator and how many accuracies they want to run,
    # e.g., `--convex 0.1 0.01 0.001 --runge_kugga3 0.1 0.01``
    parser.add_argument(
        "--convex",
        type=float,
        nargs="+",
        help="Convex integrator accuracies to try",
    )
    parser.add_argument(
        "--runge_kutta3",
        type=float,
        nargs="+",
        help="runge_kutta3 integrator accuracies to try",
    )
    parser.add_argument(
        "--implicit_euler",
        type=float,
        nargs="+",
        help="Implicit Euler integrator accuracies to try",
    )
    parser.add_argument(
        "--radau3",
        type=float,
        nargs="+",
        help="Radau3 integrator accuracies to try",
    )
    args = parser.parse_args()

    # Set up the example system
    if args.example == "gripper":
        example = gripper()
    elif args.example == "ball_on_table":
        example = ball_on_table()
    elif args.example == "double_pendulum":
        example = double_pendulum()
    elif args.example == "cylinder_hydro":
        example = cylinder_hydro()
    elif args.example == "cylinder_point":
        example = cylinder_point()
    elif args.example == "clutter":
        example = clutter()
    else:
        raise ValueError(f"Unknown example {args.example}")

    meshcat = StartMeshcat()

    # Set up data storage
    convex_accuracies = []
    convex_wall_times = []
    runge_kutta3_accuracies = []
    runge_kutta3_wall_times = []
    implicit_euler_accuracies = []
    implicit_euler_wall_times = []
    radau3_accuracies = []
    radau3_wall_times = []

    integrators_in_use = []
    for integrator in ["convex", "runge_kutta3", "implicit_euler", "radau3"]:
        if getattr(args, integrator) is None:
            continue
        for accuracy in getattr(args, integrator):
            # Run the simulation experiment
            print(f"\n==> Running {integrator} with accuracy {accuracy}...")
            try:
                _, wall_time = run_simulation(
                    example,
                    integrator=integrator,
                    accuracy=accuracy,
                    max_step_size=args.max_step_size,
                    meshcat=meshcat,
                    wait_for_meshcat=False,
                )
            except KeyboardInterrupt:
                # If the user interrupts, just skip this accuracy.
                wall_time = None
            print(f"==> Wall time: {wall_time} s")

            # Store the data
            if integrator == "convex":
                convex_accuracies.append(accuracy)
                convex_wall_times.append(wall_time)
            elif integrator == "runge_kutta3":
                runge_kutta3_accuracies.append(accuracy)
                runge_kutta3_wall_times.append(wall_time)
            elif integrator == "implicit_euler":
                implicit_euler_accuracies.append(accuracy)
                implicit_euler_wall_times.append(wall_time)
            elif integrator == "radau3":
                radau3_accuracies.append(accuracy)
                radau3_wall_times.append(wall_time)

            if integrator not in integrators_in_use:
                integrators_in_use.append(integrator)

    # Plot the data
    print("\n==> Plotting results...")

    for integrator in integrators_in_use:
        print(
            f"{integrator} accuracies:", locals()[f"{integrator}_accuracies"]
        )
        print(
            f"{integrator} wall times:", locals()[f"{integrator}_wall_times"]
        )

        plt.plot(
            locals()[f"{integrator}_accuracies"],
            locals()[f"{integrator}_wall_times"],
            "o--",
            label=integrator,
        )

    plt.xscale("log")
    plt.yscale("log")
    plt.xlabel("Accuracy")
    plt.ylabel("Wall time (s)")
    plt.gca().invert_xaxis()
    plt.legend()
    plt.title(f"{example.name}, {example.sim_time} s simulation")
    plt.grid()

    plt.show()
