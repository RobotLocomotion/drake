import argparse
from pydrake.all import *
import time
import numpy as np

##
#
# Simulate robot stacking dishes on a funky slanted dishrack. This requires
# high precision to resolve frictional contact between the plate and the
# dishrack, ensure that the plate does not pass through the thin bars, and
# resolve the high stiffnesses in the robot's high-gain controller.
#
##


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--integrator",
        type=str,
        default="convex",
        help="Integrator to use. Default: 'convex'.",
    )
    parser.add_argument(
        "--accuracy",
        type=float,
        default=0.1,
        help="Integrator accuracy. Default: 0.1.",
    )
    parser.add_argument(
        "--sim_time",
        type=float,
        default=5.0,
        help="Simulation time (in seconds). Default: 5.0.",
    )
    parser.add_argument(
        "--mbp_time_step",
        type=float,
        default=0.0,
        help="MultibodyPlant time step. Default: 0.0",
    )
    args = parser.parse_args()

    # Set up the system diagram
    meshcat = StartMeshcat()
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=args.mbp_time_step)
    parser = Parser(plant)

    # Add the dishrack
    parser.AddModelsFromUrl(
        "package://drake/examples/integrators/slanted_dishrack.xml"
    )

    # Add a plate model
    plate = parser.AddModelsFromUrl(
        "package://drake_models/dishes/plate_8in.sdf"
    )[0]

    # Add a robot arm

    # Add a controller for the robot arm

    plant.Finalize()

    # Configure default hydroelastic contact
    sg_config = SceneGraphConfig()
    sg_config.default_proximity_properties.compliance_type = "compliant"
    scene_graph.set_config(sg_config)

    # Add the visualizer
    vis_config = VisualizationConfig()
    vis_config.publish_period = 100  # very long to avoid extra publishes
    ApplyVisualizationConfig(vis_config, builder=builder, meshcat=meshcat)

    # Finalize the diagram
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)

    # Set the initial state
    q0 = np.array([0.7, 0.7, 0.0, 0.0, -0.1, 0.0, 0.5])
    plant.SetPositions(plant_context, plate, q0)

    # Set up the simulator
    simulator = Simulator(diagram, diagram_context)
    config = SimulatorConfig()
    if args.mbp_time_step == 0:
        config.integration_scheme = args.integrator
    config.accuracy = args.accuracy
    config.target_realtime_rate = 0.0
    config.publish_every_time_step = True
    ApplySimulatorConfig(config, simulator)
    simulator.Initialize()

    input("Waiting for meshcat... press [ENTER] to continue")

    # Run the sim
    meshcat.StartRecording()
    start_time = time.time()
    simulator.AdvanceTo(args.sim_time)
    wall_time = time.time() - start_time
    meshcat.StopRecording()
    meshcat.PublishRecording()

    print(f"\nWall time: {wall_time}\n")

    PrintSimulatorStatistics(simulator)

    input("Waiting for meshcat... press [ENTER] to continue")
