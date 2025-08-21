##
#
# Simulate a user-provided model in mujoco xml format.
#
##

import argparse
from pydrake.geometry import StartMeshcat, SceneGraphConfig
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import PdControllerGains
from pydrake.systems.analysis import (
    Simulator,
    SimulatorConfig,
    ApplySimulatorConfig,
    PrintSimulatorStatistics,
)
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization
import time


def run_simulation(
    xml_file,
    visualize,
    sim_time,
    hydroelastic,
    mbp_time_step,
    use_error_control,
    accuracy,
    max_time_step,
):
    # Start meshcat
    meshcat = StartMeshcat()

    # Set up the system diagram
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, mbp_time_step)
    parser = Parser(plant, scene_graph)
    parser.AddModels(xml_file)
    # Remove implicit PD actuation, if any exists
    for idx in plant.GetJointActuatorIndices():
        actuator = plant.get_joint_actuator(idx)
        if actuator.has_controller():
            actuator.set_controller_gains(PdControllerGains(p=0.0, d=0.0))
    plant.Finalize()

    if hydroelastic:
        sg_config = SceneGraphConfig()
        sg_config.default_proximity_properties.compliance_type = "compliant"
        sg_config.default_proximity_properties.slab_thickness = 0.1
        scene_graph.set_config(sg_config)

    if visualize:
        AddDefaultVisualization(builder=builder, meshcat=meshcat)

    diagram = builder.Build()

    # Initialize the simulator
    config = SimulatorConfig()
    if mbp_time_step == 0:
        config.integration_scheme = "convex"
        config.accuracy = accuracy
        config.use_error_control = use_error_control
        config.max_step_size = max_time_step

    simulator = Simulator(diagram)
    ApplySimulatorConfig(config, simulator)
    if mbp_time_step == 0:
        ci = simulator.get_mutable_integrator()
        ci.set_plant(plant)
        params = ci.get_solver_parameters()
        params.print_solver_stats = False
        params.use_dense_algebra = False
        ci.set_solver_parameters(params)
    if visualize:
        simulator.set_publish_every_time_step(True)
    else:
        simulator.set_publish_every_time_step(False)

    simulator.Initialize()

    # Wait for meshcat to be ready
    if mbp_time_step > 0:
        time_step = mbp_time_step
    else:
        if use_error_control:
            time_step = (
                f"[error controlled, acc={accuracy}, max_step={max_time_step}]"
            )
        else:
            time_step = f"{max_time_step} (convex integrator)"

    print(
        f"Simulating a {plant.num_positions()} DoF model",
        f"for {sim_time} seconds with dt={time_step}..."
    )
    if visualize:
        input("Press [ENTER] to continue...")

    # Run the simulation
    meshcat.StartRecording()
    start_time = time.time()
    simulator.AdvanceTo(sim_time)
    wall_time = time.time() - start_time
    meshcat.StopRecording()
    meshcat.PublishRecording()

    # Print some statistics
    rtr = sim_time / wall_time
    print(f"Wall time: {wall_time:.4f} seconds")
    print(f"Real-time rate: {rtr:.4f}x")

    PrintSimulatorStatistics(simulator)

    # Wait for meshcat to publish the recording
    if visualize:
        input("Press [ENTER] to exit...")

    return wall_time, rtr


if __name__ == "__main__":
    # Get system arguments for what we should do
    parser = argparse.ArgumentParser(description="Simulate a drake model.")
    parser.add_argument(
        "--mjcf",
        type=str,
        help="Path to the xml file defining the model to simulate.",
        required=True,
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Whether to visualize the simulation.",
        required=False,
    )
    parser.add_argument(
        "--sim_time",
        type=float,
        help="How long to simulate the model for (in seconds).",
        default=10.0,
        required=False,
    )
    parser.add_argument(
        "--hydroelastic",
        action="store_true",
        help="Whether to use hydroelastic contact (default is false).",
        default=False,
    )
    parser.add_argument(
        "--mbp_time_step",
        type=float,
        help="Simulator step size dt (in seconds). 0 for continuous time.",
        default=0.0,
        required=False,
    )
    parser.add_argument(
        "--use_error_control",
        action="store_true",
        help="Whether to use error control for the integrator.",
        default=False,
    )
    parser.add_argument(
        "--accuracy",
        type=float,
        help="Accuracy for the error controlled integrator (default 1e-1).",
        default=1e-3,
    )
    parser.add_argument(
        "--max_time_step",
        type=float,
        help="Maximum time step for the error controlled integrator.",
        default=0.1,
        required=False,
    )
    args = parser.parse_args()

    # Run the simulation
    run_simulation(
        args.mjcf,
        args.visualize,
        args.sim_time,
        args.hydroelastic,
        args.mbp_time_step,
        args.use_error_control,
        args.accuracy,
        args.max_time_step,
    )
