import argparse
from pydrake.all import *
import time
import numpy as np

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
        builder, time_step=args.mbp_time_step
    )

    # Add an ellipoidal spinner with offset inertial
    friction = CoulombFriction(0.2, 0.2)
    mass = 0.1
    l1, l2, l3 = 0.1, 0.02, 0.01
    unit_inertia = UnitInertia.SolidEllipsoid(l1, l2, l3)
    spatial_inertia = SpatialInertia(mass, [0, 0, 0], unit_inertia)

    spinner = plant.AddRigidBody("spinner", spatial_inertia)
    X = RigidTransform(p=[0, 0, 0], rpy=RollPitchYaw([0.0, 0.0, 0.2]))
    plant.RegisterVisualGeometry(
        spinner, X, Ellipsoid(l1, l2, l3), "visual", [0.5, 0.5, 0.5, 1]
    )
    plant.RegisterCollisionGeometry(
        spinner, X, Ellipsoid(l1, l2, l3), "collision", friction
    )

    # Add ground
    plant.RegisterCollisionGeometry(
        plant.world_body(),
        RigidTransform(p=[0, 0, -1]),
        Box(2, 2, 2),
        "ground",
        friction,
    )
    plant.Finalize()

    AddDefaultVisualization(builder=builder, meshcat=meshcat)
    diagram = builder.Build()

    # Set default contact properties
    sg_config = SceneGraphConfig()
    sg_config.default_proximity_properties.hunt_crossley_dissipation = 0.0
    sg_config.default_proximity_properties.point_stiffness = 1e7
    scene_graph.set_config(sg_config)

    # Set the initial state
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(diagram_context)

    q0 = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, l3])
    v0 = np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0])
    plant.SetPositions(plant_context, q0)
    plant.SetVelocities(plant_context, v0)

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
