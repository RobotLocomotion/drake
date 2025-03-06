import argparse
from pydrake.all import *
import numpy as np
import time

##
#
# Simulate a robot controlled with a neural network policy with random
# weights. This will stress-test our linear treatment of external control
# systems.
#
##

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--integrator",
        type=str,
        default="convex",
        help="Integrator to use, e.g., 'convex', 'implicit_euler'.",
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
        help="Simulation time (in seconds).",
    )
    parser.add_argument(
        "--mbp_time_step",
        type=float,
        default=0.0,
        help="MultibodyPlant time step (>0 for discrete simulation)",
    )
    args = parser.parse_args()

    # Set up the system diagram
    meshcat = StartMeshcat()
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=args.mbp_time_step)

    ground_box = plant.AddRigidBody(
        "ground", SpatialInertia(1, np.array([0, 0, 0]), UnitInertia(1, 1, 1)))
    X_WG = RigidTransform([0, 0, -0.05])
    ground_geometry_id = plant.RegisterCollisionGeometry(
        ground_box, RigidTransform(), Box(10, 10, 0.1), "ground",
        CoulombFriction(0.9, 0.8))
    plant.RegisterVisualGeometry(
        ground_box, RigidTransform(), Box(10, 10, 0.1), "ground",
        [0.5, 0.5, 0.5, 1.])
    plant.WeldFrames(plant.world_frame(), ground_box.body_frame(), X_WG)

    Parser(plant).AddModels(
        url="package://drake_models/atlas/atlas_convex_hull.urdf")

    plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, 0.0])
    plant.Finalize()

    nu = plant.num_actuators()
    nx = plant.num_multibody_states()
    ctrl = builder.AddSystem(MultilayerPerceptron([nx, 128, 128, nu]))
    builder.Connect(ctrl.get_output_port(), plant.get_actuation_input_port())
    builder.Connect(plant.get_state_output_port(), ctrl.get_input_port())

    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    print("Building the diagram...")
    diagram = builder.Build()
    print("done building.")

    # Set the initial state
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(context)
    q0 = plant.GetPositions(plant_context)
    q0[6] = 1.0
    plant.SetPositions(plant_context, q0)

    # Set random weights for the controller
    ctrl_context = ctrl.GetMyMutableContextFromRoot(context)
    ctrl.SetRandomContext(ctrl_context, RandomGenerator(0))

    # Set up the simulator
    config = SimulatorConfig()
    config.integration_scheme = args.integrator
    config.accuracy = args.accuracy
    config.target_realtime_rate = 0.0
    config.use_error_control = True
    config.publish_every_time_step = True

    simulator = Simulator(diagram, context)
    ApplySimulatorConfig(config, simulator)
    simulator.Initialize()

    print(f"Running with {args.integrator} at accuracy = {args.accuracy}.")
    input("Waiting for meshcat... press [ENTER] to continue")

    # Run the sim
    meshcat.StartRecording()
    st = time.time()
    simulator.AdvanceTo(args.sim_time)
    wall_time = time.time() - st
    meshcat.StopRecording()
    meshcat.PublishRecording()

    print(f"\nWall time: {wall_time}\n")

    # Print a summary of solver statistics
    PrintSimulatorStatistics(simulator)
