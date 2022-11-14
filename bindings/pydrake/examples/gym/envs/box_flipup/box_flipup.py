import gym
import numpy as np
import os.path

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    Box,
    ConstantVectorSource,
    ContactVisualizer,
    ContactVisualizerParams,
    DiagramBuilder,
    EventStatus,
    FindResourceOrThrow,
    FixedOffsetFrame,
    InverseDynamicsController,
    LeafSystem,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    MultibodyPlant,
    MultibodyPositionToGeometryPose,
    Multiplexer,
    Parser,
    PassThrough,
    PlanarJoint,
    PrismaticJoint,
    RandomGenerator,
    Rgba,
    RigidTransform,
    RotationMatrix,
    SceneGraph,
    Simulator,
    SpatialInertia,
    Sphere,
    UnitInertia,
    Variable,
)

from pydrake.gym.drake_gym import DrakeGymEnv
from pydrake.examples.gym.scenario_helpers import (
    AddShape,
    SetColor,
    SetTransparency,
)


def AddPlanarBinAndSimpleBox(plant,
                             mass=1.0,
                             mu=1.0,
                             width=0.2,
                             depth=0.05,
                             height=0.3):
    parser = Parser(plant)
    sdf_location = FindResourceOrThrow(
        "drake/bindings/pydrake/examples/gym/models/planar_bin.sdf")
    bin = parser.AddModelFromFile(sdf_location)
    plant.WeldFrames(
        plant.world_frame(), plant.GetFrameByName("bin_base", bin),
        RigidTransform(RotationMatrix.MakeZRotation(np.pi / 2.0),
                       [0, 0, -0.015]))

    planar_joint_frame = plant.AddFrame(
        FixedOffsetFrame(
            "planar_joint_frame", plant.world_frame(),
            RigidTransform(RotationMatrix.MakeXRotation(np.pi / 2))))

    # TODO(russt): make this a *random* box?
    # TODO(russt): move random box to a shared py file.
    box_instance = AddShape(plant, Box(width, depth, height), "box", mass, mu)
    box_joint = plant.AddJoint(
        PlanarJoint("box", planar_joint_frame,
                    plant.GetFrameByName("box", box_instance)))
    box_joint.set_position_limits([-.5, -.1, -np.pi], [.5, .3, np.pi])
    box_joint.set_velocity_limits([-2, -2, -2], [2, 2, 2])
    box_joint.set_default_translation([0, depth / 2.0])
    return box_instance


def AddPointFinger(plant):
    finger = AddShape(plant, Sphere(0.01), "finger", color=[.9, .5, .5, 1.0])
    false_body1 = plant.AddRigidBody(
        "false_body1", finger, SpatialInertia(0, [0, 0, 0],
                                              UnitInertia(0, 0, 0)))
    finger_x = plant.AddJoint(
        PrismaticJoint("finger_x", plant.world_frame(),
                       plant.GetFrameByName("false_body1"),
                       [1, 0, 0], -.3, .3))
    finger_x.set_position_limits([-.5], [.5])
    finger_x.set_velocity_limits([-2], [2])
    plant.AddJointActuator("finger_x", finger_x)
    finger_z = plant.AddJoint(
        PrismaticJoint("finger_z", plant.GetFrameByName("false_body1"),
                       plant.GetFrameByName("finger"), [0, 0, 1], 0.0, 0.5))
    finger_z.set_default_translation(0.25)
    finger_z.set_position_limits([-.1], [.3])
    finger_z.set_velocity_limits([-2], [2])
    plant.AddJointActuator("finger_z", finger_z)

    return finger


def make_box_flipup(generator,
                    observations="state",
                    meshcat=None,
                    time_limit=10):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    # TODO(russt): randomize parameters.
    box = AddPlanarBinAndSimpleBox(plant)
    finger = AddPointFinger(plant)
    plant.Finalize()
    plant.set_name("plant")
    SetTransparency(scene_graph, alpha=0.5, source_id=plant.get_source_id())
    controller_plant = MultibodyPlant(time_step=0.005)
    AddPointFinger(controller_plant)

    if meshcat:
        MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
        meshcat.Set2dRenderMode(xmin=-.35, xmax=.35, ymin=-0.1, ymax=0.3)
        ContactVisualizer.AddToBuilder(
            builder, plant, meshcat,
            ContactVisualizerParams(radius=0.005, newtons_per_meter=40.0))

        # Use the controller plant to visualize the set point geometry.
        controller_scene_graph = builder.AddSystem(SceneGraph())
        controller_plant.RegisterAsSourceForSceneGraph(controller_scene_graph)
        SetColor(controller_scene_graph,
                 color=[1.0, 165.0 / 255, 0.0, 1.0],
                 source_id=controller_plant.get_source_id())
        controller_vis = MeshcatVisualizer.AddToBuilder(
            builder, controller_scene_graph, meshcat,
            MeshcatVisualizerParams(prefix="controller"))
        controller_vis.set_name("controller meshcat")

    controller_plant.Finalize()

    # Stiffness control.  (For a point finger with unit mass, the
    # InverseDynamicsController is identical)
    N = controller_plant.num_positions()
    kp = [100] * N
    ki = [1] * N
    kd = [2 * np.sqrt(kp[0])] * N
    controller = builder.AddSystem(
        InverseDynamicsController(controller_plant, kp, ki, kd, False))
    builder.Connect(plant.get_state_output_port(finger),
                    controller.get_input_port_estimated_state())

    actions = builder.AddSystem(PassThrough(N))
    positions_to_state = builder.AddSystem(Multiplexer([N, N]))
    builder.Connect(actions.get_output_port(),
                    positions_to_state.get_input_port(0))
    zeros = builder.AddSystem(ConstantVectorSource([0] * N))
    builder.Connect(zeros.get_output_port(),
                    positions_to_state.get_input_port(1))
    builder.Connect(positions_to_state.get_output_port(),
                    controller.get_input_port_desired_state())
    builder.Connect(controller.get_output_port(),
                    plant.get_actuation_input_port())
    if meshcat:
        positions_to_poses = builder.AddSystem(
            MultibodyPositionToGeometryPose(controller_plant))
        builder.Connect(
            positions_to_poses.get_output_port(),
            controller_scene_graph.get_source_pose_port(
                controller_plant.get_source_id()))

    builder.ExportInput(actions.get_input_port(), "actions")
    if observations == "state":
        builder.ExportOutput(plant.get_state_output_port(), "observations")
    # TODO(russt): Add 'time', and 'keypoints'
    else:
        raise ValueError("observations must be one of ['state']")

    class RewardSystem(LeafSystem):

        def __init__(self):
            LeafSystem.__init__(self)
            self.DeclareVectorInputPort("box_state", 6)
            self.DeclareVectorInputPort("finger_state", 4)
            self.DeclareVectorInputPort("actions", 2)
            self.DeclareVectorOutputPort("reward", 1, self.CalcReward)

        def CalcReward(self, context, output):
            box_state = self.get_input_port(0).Eval(context)
            finger_state = self.get_input_port(1).Eval(context)
            actions = self.get_input_port(2).Eval(context)

            angle_from_vertical = (box_state[2] % np.pi) - np.pi / 2
            cost = 2 * angle_from_vertical**2  # box angle
            cost += 0.1 * box_state[5]**2  # box velocity
            effort = actions - finger_state[:2]
            cost += 0.1 * effort.dot(effort)  # effort
            # finger velocity
            cost += 0.1 * finger_state[2:].dot(finger_state[2:])
            # Add 10 to make rewards positive (to avoid rewarding simulator
            # crashes).
            output[0] = 10 - cost

    reward = builder.AddSystem(RewardSystem())
    builder.Connect(plant.get_state_output_port(box), reward.get_input_port(0))
    builder.Connect(plant.get_state_output_port(finger),
                    reward.get_input_port(1))
    builder.Connect(actions.get_output_port(), reward.get_input_port(2))
    builder.ExportOutput(reward.get_output_port(), "reward")

    # Set random state distributions.
    uniform_random = Variable(name="uniform_random",
                              type=Variable.Type.RANDOM_UNIFORM)
    box_joint = plant.GetJointByName("box")
    x, y = box_joint.get_default_translation()
    box_joint.set_random_pose_distribution(
        [.2 * uniform_random - .1 + x, y],
        0)

    diagram = builder.Build()
    simulator = Simulator(diagram)

    # Termination conditions:
    def monitor(context):
        if context.get_time() > time_limit:
            return EventStatus.ReachedTermination(diagram, "time limit")
        return EventStatus.Succeeded()

    simulator.set_monitor(monitor)
    return simulator


def BoxFlipUpEnv(observations="state", meshcat=None, time_limit=10):
    simulator = make_box_flipup(RandomGenerator(),
                                observations,
                                meshcat=meshcat,
                                time_limit=time_limit)
    action_space = gym.spaces.Box(low=np.array([-.5, -0.1]),
                                  high=np.array([.5, 0.6]),
                                  dtype=np.float64)

    plant = simulator.get_system().GetSubsystemByName("plant")

    # It is unsound to use raw MBP dof limits as observation bounds, because
    # most simulations will violate those limits in practice (in collision, or
    # due to gravity, or in general because no constraint force is incurred
    # except in violation).  However we don't have any other better limits
    # here.  So we broaden the limits by a fixed offset and hope for the best.
    NUM_DOFS = 5
    POSITION_LIMIT_TOLERANCE = np.full((NUM_DOFS,), 0.1)
    VELOCITY_LIMIT_TOLERANCE = np.full((NUM_DOFS,), 0.5)
    if observations == "state":
        low = np.concatenate(
            (plant.GetPositionLowerLimits() - POSITION_LIMIT_TOLERANCE,
             plant.GetVelocityLowerLimits() - VELOCITY_LIMIT_TOLERANCE))
        high = np.concatenate(
            (plant.GetPositionUpperLimits() + POSITION_LIMIT_TOLERANCE,
             plant.GetVelocityUpperLimits() + VELOCITY_LIMIT_TOLERANCE))
        observation_space = gym.spaces.Box(
            low=np.asarray(low),
            high=np.asarray(high),
            dtype=np.float64)

    env = DrakeGymEnv(simulator=simulator,
                      time_step=0.1,
                      action_space=action_space,
                      observation_space=observation_space,
                      reward="reward",
                      action_port_id="actions",
                      observation_port_id="observations")
    return env
