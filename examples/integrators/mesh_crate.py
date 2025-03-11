import argparse
from pydrake.all import *
import time
import numpy as np

##
#
# Simulate a complicated system with a mesh crate.
#
##


def get_random_configuration(z_height=None):
    """Sample a random configuration for an object above the crate."""
    quat = np.random.uniform(-1, 1, 4)
    quat /= np.linalg.norm(quat)
    pos = np.random.uniform(-0.2, 0.2, 3)
    if z_height is not None:
        pos[2] = z_height
    else:
        pos[2] = np.random.uniform(0.0, 0.5)
    return np.concatenate((quat, pos))


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
        default=10.0,
        help="Simulation time (in seconds). Default: 10.0.",
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

    # Add the mesh crate
    parser.AddModelsFromUrl(
        "package://drake/examples/integrators/mesh_crate.xml"
    )

    # Add clutter
    model_urls = [
        "package://drake_models/ycb/003_cracker_box.sdf",
        "package://drake_models/dishes/plate_8in.sdf",
        "package://drake_models/ycb/004_sugar_box.sdf",
        "package://drake_models/ycb/005_tomato_soup_can.sdf",
        "package://drake_models/ycb/006_mustard_bottle.sdf",
        "package://drake_models/ycb/009_gelatin_box.sdf",
        "package://drake_models/ycb/010_potted_meat_can.sdf",
        "package://drake_models/veggies/yellow_bell_pepper_no_stem_low.sdf",
    ]

    models = []
    for model_url in model_urls:
        models.append(parser.AddModelsFromUrl(model_url)[0])

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
    np.random.seed(0)

    for i in range(len(models)):
        q0 = get_random_configuration(z_height=0.1 + 0.2 * i)
        plant.SetPositions(plant_context, models[i], q0)

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
