import argparse
from pydrake.all import *

import time
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass

##
#
# Compare different integration schemes on a few toy examples.
#
##


@dataclass
class SimulationExample:
    """A little container for setting up different examples."""

    name: str
    url: str
    use_hydroelastic: bool
    initial_state: np.array
    sim_time: float


def gripper():
    """A fake "gripper", where an object is wedged between two fixed joints."""
    name = "Gripper"
    url = "package://drake/examples/integrators/gripper.xml"
    use_hydroelastic = True
    initial_state = np.array([0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    sim_time = 2.0
    return SimulationExample(
        name, url, use_hydroelastic, initial_state, sim_time
    )


def ball_on_table():
    """A sphere is dropped on a table with some initial horizontal velocity."""
    name = "Ball on table"
    url = "package://drake/examples/integrators/ball_on_table.xml"
    use_hydroelastic = False
    initial_state = np.array(
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    )
    sim_time = 1.0
    return SimulationExample(
        name, url, use_hydroelastic, initial_state, sim_time
    )


def double_pendulum():
    """A simple double pendulum (no contact)."""
    name = "Double Pendulum"
    url = "package://drake/examples/integrators/double_pendulum.xml"
    use_hydroelastic = False
    initial_state = np.array([3.0, 0.1, 0.0, 0.0])
    sim_time = 2.0
    return SimulationExample(
        name, url, use_hydroelastic, initial_state, sim_time
    )


def cylinder_hydro():
    """A cylinder with hydroelastic contact is dropped on the table."""
    name = "Cylinder w/ hydroelastic"
    url = "package://drake/examples/integrators/cylinder.xml"
    use_hydroelastic = True
    initial_state = np.array(
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    )
    sim_time = 1.0
    return SimulationExample(
        name, url, use_hydroelastic, initial_state, sim_time
    )


def cylinder_point():
    """A cylinder with point contact is dropped on the table."""
    name = "Cylinder w/ point contact"
    url = "package://drake/examples/integrators/cylinder.xml"
    use_hydroelastic = False
    initial_state = np.array(
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    )
    sim_time = 1.0
    return SimulationExample(
        name, url, use_hydroelastic, initial_state, sim_time
    )


def clutter():
    """Several spheres fall into a box."""
    name = "Clutter"
    url = "package://drake/examples/integrators/clutter.xml"
    use_hydroelastic = False
    initial_state = np.array(
        [
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]
    )
    sim_time = 3.0
    return SimulationExample(
        name, url, use_hydroelastic, initial_state, sim_time
    )


def plate_and_spatula():
    """A spatula is dropped onto a plate."""
    name = "Plate and spatula"
    url = "package://drake/examples/integrators/plate_and_spatula.sdf"
    use_hydroelastic = True
    initial_state = np.array([
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02,   # plate pos
        1.0, 0.0, 0.0, 0.0, -0.05, 0.0, 0.08,  # spatula pos
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,         # plate vel
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,         # spatula vel
    ])
    sim_time = 2.0
    return SimulationExample(
        name, url, use_hydroelastic, initial_state, sim_time
    )


def create_scene(
    url: str,
    time_step: float,
    meshcat: Meshcat,
    hydroelastic: bool = False,
):
    """
    Set up a drake system dyagram

    Args:
        xml: mjcf robot description.
        time_step: dt for MultibodyPlant.
        hydroelastic: whether to use hydroelastic contact.
        meshcat: meshcat instance for visualization.

    Returns:
        The system diagram, the MbP within that diagram, and the logger used to
        keep track of time steps.
    """
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=time_step
    )

    parser = Parser(plant)
    parser.AddModels(url=url)
    if time_step > 0:
        plant.set_discrete_contact_approximation(
            DiscreteContactApproximation.kLagged
        )
    plant.Finalize()

    if hydroelastic:
        sg_config = SceneGraphConfig()
        sg_config.default_proximity_properties.compliance_type = "compliant"
        scene_graph.set_config(sg_config)

    vis_config = VisualizationConfig()
    vis_config.publish_period = 100  # very long to avoid extra publishes
    ApplyVisualizationConfig(vis_config, builder=builder, meshcat=meshcat)

    logger = LogVectorOutput(
        plant.get_state_output_port(),
        builder,
        publish_triggers={TriggerType.kForced},
        publish_period=0,
    )
    logger.set_name("logger")

    diagram = builder.Build()
    return diagram, plant, logger


def run_simulation(
    example: SimulationExample,
    integrator: str,
    accuracy: float,
    max_step_size: float,
    meshcat: Meshcat,
    wait_for_meshcat: bool = True,
):
    """
    Run a short simulation, and report the time-steps used throughout.

    Args:
        example: container defining the scenario to simulate.
        integrator: which integration strategy to use ("implicit_euler",
            "runge_kutta3", "convex", "discrete").
        accuracy: the desired accuracy (ignored for "discrete").
        max_step_size: the maximum (and initial) timestep dt.
        meshcat: meshcat instance for visualization.
        wait_for_meshcat: whether to wait for meshcat load.

    Returns:
        Timesteps (dt) throughout the simulation, and the wall-clock time.
    """
    url = example.url
    use_hydroelastic = example.use_hydroelastic
    initial_state = example.initial_state
    sim_time = example.sim_time

    # We can use a more standard simulation setup and rely on a logger to
    # tell use the time step information. Note that in this case enabling
    # visualization messes with the time step report though.

    # Configure Drake's built-in error controlled integration
    config = SimulatorConfig()
    if integrator != "discrete":
        config.integration_scheme = integrator
    config.max_step_size = max_step_size
    config.accuracy = accuracy
    config.target_realtime_rate = 0.0
    config.use_error_control = True
    config.publish_every_time_step = True

    # Set up the system diagram and initial condition
    if integrator == "discrete":
        time_step = max_step_size
    else:
        time_step = 0.0
    diagram, plant, logger = create_scene(
        url, time_step, meshcat, use_hydroelastic
    )
    context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, context)

    plant.SetPositionsAndVelocities(plant_context, initial_state)

    simulator = Simulator(diagram, context)
    ApplySimulatorConfig(config, simulator)
    simulator.Initialize()

    print(f"Running the {example.name} example with {integrator} integrator.")
    if wait_for_meshcat:
        input("Waiting for meshcat... [ENTER] to continue")

    # Simulate
    meshcat.StartRecording()
    start_time = time.time()
    simulator.AdvanceTo(sim_time)
    wall_time = time.time() - start_time
    meshcat.StopRecording()
    meshcat.PublishRecording()

    print(f"\nWall clock time: {wall_time}\n")
    PrintSimulatorStatistics(simulator)

    # Get timesteps from the logger
    log = logger.FindLog(context)
    times = log.sample_times()
    timesteps = times[1:] - times[0:-1]

    return np.asarray(timesteps), wall_time


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--example",
        type=str,
        default="clutter",
        help=(
            "Which example to run. One of: gripper, ball_on_table, "
            "double_pendulum, cylinder_hydro, cylinder_point, clutter. "
        ),
    )
    parser.add_argument(
        "--integrator",
        type=str,
        default="convex",
        help=(
            "Integrator to use, e.g., implicit_euler, runge_kutta3, convex, "
            "discrete. Default: convex."
        ),
    )
    parser.add_argument(
        "--accuracy",
        type=float,
        default=0.1,
        help="Integrator accuracy (ignored for discrete). Default: 0.1.",
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
    parser.add_argument(
        "--plot",
        action="store_true",
        help=(
            "Whether to make plots of the step size over time. Default: "
            "False."
        ),
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
    elif args.example == "plate_and_spatula":
        example = plate_and_spatula()
    else:
        raise ValueError(f"Unknown example {args.example}")

    meshcat = StartMeshcat()

    time_steps, _ = run_simulation(
        example,
        args.integrator,
        args.accuracy,
        max_step_size=args.max_step_size,
        meshcat=meshcat,
    )

    if args.plot:
        times = np.cumsum(time_steps)
        plt.title(
            (
                f"{example.name} | {args.integrator} integrator | "
                f"accuracy = {args.accuracy}"
            )
        )
        plt.plot(times, time_steps, "o")
        plt.ylim(1e-10, 1e0)
        plt.yscale("log")
        plt.xlabel("time (s)")
        plt.ylabel("step size (s)")
        plt.show()
