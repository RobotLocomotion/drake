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


def add_arm_with_gripper(plant, parser):
    """Add a UR3 arm with a gripper to the given plant."""
    arm = parser.AddModelsFromUrl(
     "package://drake_models/ur_description/urdf/ur3e_cylinders_collision.urdf"
    )[0]
    gripper = parser.AddModelsFromUrl(
     "package://drake_models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf"
    )[0]

    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("ur_base_link", arm),
        RigidTransform([0.5, 0.0, 0.0]),
    )
    plant.WeldFrames(
        plant.GetFrameByName("ur_ee_link", arm),
        plant.GetFrameByName("body", gripper),
        RigidTransform(
            p=[0.04, 0.0, 0.0], rpy=RollPitchYaw([0, 0, -np.pi / 2])
        ),
    )
    return arm, gripper


class TargetStateProvider(LeafSystem):
    """A simple system that provides a sequence of target configurations,
    purely based on time.
    """

    def __init__(self):
        super().__init__()
        self.DeclareVectorOutputPort(
            name="x_nom", size=16, calc=self.CalcOutput
        )

    def CalcOutput(self, context, output):
        t = context.get_time()

        q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, np.pi / 2, 0.0, 0.0])
        q_move = np.array(
            [0.0, -np.pi / 2, -1.0, 2.6, -np.pi / 2, 0.0, 0.0, 0.0]
        )
        q_drop = np.array(
            [0.0, -np.pi / 2, -1.0, 2.6, -np.pi / 2, 0.0, -0.01, 0.01]
        )

        if t < 1.0:
            # Grip the plate in the initial configuration
            q_nom = q_start
        elif t < 5.0:
            # Move over the dishrack
            q_nom = q_move
        elif t < 6.0:
            # Drop the plate
            q_nom = q_drop
        else:
            # Move back to the initial configuration
            q_nom = q_start

        v_nom = np.zeros(8)
        output.SetFromVector(np.concatenate((q_nom, v_nom)))


class ArmStateEstimator(LeafSystem):
    """Get the state of the arm and the gripper only."""

    def __init__(self):
        super().__init__()
        self.arm_input_port = self.DeclareVectorInputPort("arm_state", size=12)
        self.gripper_input_port = self.DeclareVectorInputPort(
            "gripper_state", size=4
        )
        self.DeclareVectorOutputPort(
            name="state", size=16, calc=self.CalcOutput
        )

    def CalcOutput(self, context, output):
        x_arm = self.arm_input_port.Eval(context)
        x_gripper = self.gripper_input_port.Eval(context)
        q_arm = x_arm[:6]
        v_arm = x_arm[6:]
        q_gripper = x_gripper[:2]
        v_gripper = x_gripper[2:]
        x = np.concatenate((q_arm, q_gripper, v_arm, v_gripper))
        output.SetFromVector(x)


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
        builder, time_step=args.mbp_time_step
    )
    parser = Parser(plant)

    # Add the dishrack
    parser.AddModelsFromUrl(
        "package://drake/examples/integrators/slanted_dishrack.xml"
    )

    # Add a plate model
    plate = parser.AddModelsFromUrl(
        "package://drake_models/dishes/plate_8in.sdf"
    )[0]

    # Add a robot arm model
    arm, gripper = add_arm_with_gripper(plant, parser)

    plant.Finalize()

    # Add a high-gain controller for the robot arm
    kp = 1e6 * np.ones(plant.num_actuators())
    kd = 1e6 * np.ones(plant.num_actuators())
    ki = 10 * np.ones(plant.num_actuators())

    ctrl_plant = MultibodyPlant(0.0)
    add_arm_with_gripper(ctrl_plant, Parser(ctrl_plant))
    ctrl_plant.Finalize()

    controller = builder.AddSystem(
        InverseDynamicsController(
            ctrl_plant, kp=kp, ki=ki, kd=kd, has_reference_acceleration=False
        )
    )

    # Add a high-level planner that provides target configurations for the arm
    planner = builder.AddSystem(TargetStateProvider())

    # Connect the controller
    arm_state_estimator = builder.AddSystem(ArmStateEstimator())
    builder.Connect(
        plant.get_state_output_port(arm),
        arm_state_estimator.arm_input_port,
    )
    builder.Connect(
        plant.get_state_output_port(gripper),
        arm_state_estimator.gripper_input_port,
    )

    builder.Connect(
        arm_state_estimator.get_output_port(),
        controller.get_input_port_estimated_state(),
    )
    builder.Connect(
        planner.get_output_port(),
        controller.get_input_port_desired_state(),
    )
    builder.Connect(
        controller.get_output_port_control(),
        plant.get_actuation_input_port(),
    )

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
    q0_plate = np.array([1.0, 0.0, 0.0, 0.0, 0.95, 0.41, 0.06])
    plant.SetPositions(plant_context, plate, q0_plate)

    q0_gripper = np.array([-0.005, 0.005])
    plant.SetPositions(plant_context, gripper, q0_gripper)

    q0_arm = np.array([0.0, 0.0, 0.0, 0.0, 0.0, np.pi / 2])
    plant.SetPositions(plant_context, arm, q0_arm)

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
