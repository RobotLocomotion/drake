import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.cpp_param import List
from pydrake.common.value import Value
from pydrake.examples.gym.named_view_helpers import (
    MakeNamedViewActuation,
    MakeNamedViewPositions,
    MakeNamedViewState,
)
from pydrake.geometry import (
    ClippingRange,
    ColorRenderCamera,
    DepthRange,
    DepthRenderCamera,
    MakeRenderEngineVtk,
    MeshcatVisualizer,
    RenderCameraCore,
    RenderEngineVtkParams,
)
from pydrake.gym import DrakeGymEnv
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.math import SpatialForce
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlant,
    ExternallyAppliedSpatialForce_,
    MultibodyPlant,
    MultibodyPlantConfig,
)
from pydrake.systems.analysis import Simulator
from pydrake.systems.drawing import plot_graphviz, plot_system_graphviz
from pydrake.systems.framework import (
    DiagramBuilder,
    EventStatus,
    LeafSystem,
    PublishEvent,
)
from pydrake.systems.primitives import (
    ConstantVectorSource,
    Multiplexer,
    PassThrough,
)
from pydrake.systems.sensors import CameraInfo, RgbdSensor


# Gym parameters.
sim_time_step = 0.01
gym_time_step = 0.05
controller_time_step = 0.01
gym_time_limit = 5
drake_contact_models = ['point', 'hydroelastic_with_fallback']
contact_model = drake_contact_models[0]
drake_contact_approximations = ['sap', 'tamsi', 'similar', 'lagged']
contact_approximation = drake_contact_approximations[0]


def AddAgent(plant):
    parser = Parser(plant)
    model_file = FindResourceOrThrow(
        "drake/bindings/pydrake/examples/gym/models/cartpole_BSA.sdf")
    agent, = parser.AddModels(model_file)
    return agent


def make_sim(meshcat=None,
             time_limit=5,
             debug=False,
             obs_noise=False,
             monitoring_camera=False,
             add_disturbances=False):

    builder = DiagramBuilder()

    multibody_plant_config = MultibodyPlantConfig(
        time_step=sim_time_step,
        contact_model=contact_model,
        discrete_contact_approximation=contact_approximation,
        )

    plant, scene_graph = AddMultibodyPlant(multibody_plant_config, builder)

    # Add assets to the plant.
    agent = AddAgent(plant)
    plant.Finalize()
    plant.set_name("plant")

    # Add assets to the controller plant.
    controller_plant = MultibodyPlant(time_step=controller_time_step)
    AddAgent(controller_plant)

    if meshcat:
        MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    # Finalize the plant.
    controller_plant.Finalize()
    controller_plant.set_name("controller_plant")

    # Extract the controller plant information.
    ns = controller_plant.num_multibody_states()
    nv = controller_plant.num_velocities()
    na = controller_plant.num_actuators()
    nj = controller_plant.num_joints()
    npos = controller_plant.num_positions()

    # Make NamedViews
    state_view = MakeNamedViewState(controller_plant, "States")
    position_view = MakeNamedViewPositions(controller_plant, "Position")
    actuation_view = MakeNamedViewActuation(controller_plant, "Actuation")

    if debug:
        print(f'\nNumber of position: {npos},',
              f'Number of velocities: {nv},',
              f'Number of actuators: {na},',
              f'Number of joints: {nj},',
              f'Number of multibody states: {ns}')
        print("State view: ", state_view(np.ones(ns)))
        print("Position view: ", position_view(np.ones(npos)))
        print("Actuation view: ", actuation_view(np.ones(na)), '\n')

        # Visualize the plant.
        plt.figure()
        plot_graphviz(plant.GetTopologyGraphvizString())
        plt.plot(1)
        plt.show(block=False)

    # Actions are positions sent to plant.
    actuation = builder.AddSystem(Multiplexer([1, 1]))
    prismatic_actuation_force = builder.AddSystem(PassThrough(1))
    # Zero torque to the revolute joint --it is underactuated.
    revolute_actuation_torque = builder.AddSystem(ConstantVectorSource([0]))
    builder.Connect(revolute_actuation_torque.get_output_port(),
                    actuation.get_input_port(1))
    builder.Connect(prismatic_actuation_force.get_output_port(),
                    actuation.get_input_port(0))
    builder.Connect(actuation.get_output_port(),
                    plant.get_actuation_input_port(agent))
    builder.ExportInput(prismatic_actuation_force.get_input_port(), "actions")

    class ObservationPublisher(LeafSystem):
        def __init__(self, noise=False):
            LeafSystem.__init__(self)
            self.ns = plant.num_multibody_states()
            self.DeclareVectorInputPort("plant_states", self.ns)
            self.DeclareVectorOutputPort("observations", self.ns, self.CalcObs)
            self.noise = noise

        def CalcObs(self, context, output):
            plant_state = self.get_input_port(0).Eval(context)
            if self.noise:
                plant_state += np.random.uniform(low=-0.01,
                                                 high=0.01,
                                                 size=self.ns)
            output.set_value(plant_state)

    obs_pub = builder.AddSystem(ObservationPublisher(noise=obs_noise))

    builder.Connect(plant.get_state_output_port(), obs_pub.get_input_port(0))
    builder.ExportOutput(obs_pub.get_output_port(), "observations")

    class RewardSystem(LeafSystem):
        def __init__(self):
            LeafSystem.__init__(self)
            # The state port is not used.
            ns = plant.num_multibody_states()
            self.DeclareVectorInputPort("state", ns)
            self.DeclareVectorOutputPort("reward", 1, self.CalcReward)

        def CalcReward(self, context, output):
            reward = 1
            output[0] = reward

    reward = builder.AddSystem(RewardSystem())
    builder.Connect(plant.get_state_output_port(agent),
                    reward.get_input_port(0))
    builder.ExportOutput(reward.get_output_port(), "reward")

    if monitoring_camera:
        # Adds an overhead camera.
        # This is useful for logging videos of rollout evaluation.
        scene_graph.AddRenderer(
            "renderer", MakeRenderEngineVtk(RenderEngineVtkParams()))
        color_camera = ColorRenderCamera(
            RenderCameraCore(
                "renderer",
                CameraInfo(
                    width=640,
                    height=480,
                    fov_y=np.pi/4),
                ClippingRange(0.01, 10.0),
                RigidTransform()
            ), False)
        depth_camera = DepthRenderCamera(color_camera.core(),
                                         DepthRange(0.01, 10.0))
        X_PB = RigidTransform(RollPitchYaw(-np.pi/2, 0, 0),
                              np.array([0, -2.5, 0.4]))
        rgbd_camera = builder.AddSystem(
            RgbdSensor(parent_id=scene_graph.world_frame_id(),
                       X_PB=X_PB,
                       color_camera=color_camera,
                       depth_camera=depth_camera))
        builder.Connect(scene_graph.get_query_output_port(),
                        rgbd_camera.query_object_input_port())
        builder.ExportOutput(
            rgbd_camera.color_image_output_port(), "color_image")

    class DisturbanceGenerator(LeafSystem):
        def __init__(self, plant, force_mag, period, duration):
            # Applies a random force [-force_mag, force_mag] at
            # the COM of the Pole body in the x direction every
            # period seconds for a given duration.
            LeafSystem.__init__(self)
            forces_cls = Value[List[ExternallyAppliedSpatialForce_[float]]]
            self.DeclareAbstractOutputPort("spatial_forces",
                                           lambda: forces_cls(),
                                           self.CalcDisturbances)
            self.plant = plant
            self.pole_body = self.plant.GetBodyByName("Pole")
            self.force_mag = force_mag
            assert period > duration, (
                f"period: {period} must be larger than duration: {duration}")
            self.period = period
            self.duration = duration

        def CalcDisturbances(self, context, spatial_forces_vector):
            # Apply a force at COM of the Pole body.
            force = ExternallyAppliedSpatialForce_[float]()
            force.body_index = self.pole_body.index()
            force.p_BoBq_B = self.pole_body.default_com()
            y = context.get_time() % self.period
            if not ((y >= 0) and (y <= (self.period - self.duration))):
                spatial_force = SpatialForce(
                    tau=[0, 0, 0],
                    f=[np.random.uniform(
                        low=-self.force_mag,
                        high=self.force_mag),
                       0, 0])
            else:
                spatial_force = SpatialForce(
                    tau=[0, 0, 0],
                    f=[0, 0, 0])
            force.F_Bq_W = spatial_force
            spatial_forces_vector.set_value([force])

    if add_disturbances:
        # Applies a force of 1N every 1s
        # for 0.1s at the COM of the Pole body.
        disturbance_generator = builder.AddSystem(
            DisturbanceGenerator(
                plant=plant, force_mag=1,
                period=1, duration=0.1))
        builder.Connect(disturbance_generator.get_output_port(),
                        plant.get_applied_spatial_force_input_port())

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.Initialize()

    def monitor(context, state_view=state_view):
        '''
        Monitors the simulation for episode end conditions.
        '''
        plant_context = plant.GetMyContextFromRoot(context)
        state = plant.GetOutputPort("state").Eval(plant_context)
        s = state_view(state)

        # Truncation: the episode duration reaches the time limit.
        if context.get_time() > time_limit:
            if debug:
                print("Episode reached time limit.")
            return EventStatus.ReachedTermination(
                diagram,
                "time limit")

        # Termination: The pole angle exceeded +-0.2 rad.
        if abs(s.PolePin_q) > 0.2:
            if debug:
                print("Pole angle exceeded +-0.2 rad.")
            return EventStatus.ReachedTermination(
                diagram,
                "pole angle exceeded +-0.2 rad")

        # Termination: Cart position exceeded +-2.4 m.
        if abs(s.CartSlider_x) > 2.4:
            if debug:
                print("Cart position exceeded +-2.4 m.")
            return EventStatus.ReachedTermination(
                diagram,
                "cart position exceeded +-2.4 m")

        return EventStatus.Succeeded()

    simulator.set_monitor(monitor)

    if debug:
        # Visualize the controller plant and diagram.
        plt.figure()
        plot_graphviz(controller_plant.GetTopologyGraphvizString())
        plt.figure()
        plot_system_graphviz(diagram, max_depth=2)
        plt.plot(1)
        plt.show(block=False)

    return simulator


def reset_handler(simulator, diagram_context, seed):
    '''
    Provides an easy method for domain randomization.
    '''

    # Set the seed.
    np.random.seed(seed)

    # Randomize the initial position of the joints.
    home_positions = [
        ('CartSlider', np.random.uniform(low=-.1, high=0.1)),
        ('PolePin', np.random.uniform(low=-.15, high=0.15)),
    ]

    # Randomize the initial velocities of the PolePin joint.
    home_velocities = [
        ('PolePin', np.random.uniform(low=-.1, high=0.1))
    ]

    # Randomize the mass of the Pole by adding a mass offset.
    home_body_mass_offset = [
        ('Pole', np.random.uniform(low=-0.05, high=0.05))
    ]

    diagram = simulator.get_system()
    plant = diagram.GetSubsystemByName("plant")
    plant_context = diagram.GetMutableSubsystemContext(plant,
                                                       diagram_context)

    # Ensure the positions are within the joint limits.
    for pair in home_positions:
        joint = plant.GetJointByName(pair[0])
        if joint.type_name() == "revolute":
            joint.set_angle(plant_context,
                            np.clip(pair[1],
                                    joint.position_lower_limit(),
                                    joint.position_upper_limit()
                                    )
                            )
        if joint.type_name() == "prismatic":
            joint.set_translation(plant_context,
                                  np.clip(pair[1],
                                          joint.position_lower_limit(),
                                          joint.position_upper_limit()
                                          )
                                  )
    for pair in home_velocities:
        joint = plant.GetJointByName(pair[0])
        if joint.type_name() == "revolute":
            joint.set_angular_rate(plant_context,
                                   np.clip(pair[1],
                                           joint.velocity_lower_limit(),
                                           joint.velocity_upper_limit()
                                           )
                                   )
    for pair in home_body_mass_offset:
        body = plant.GetBodyByName(pair[0])
        mass = body.get_mass(plant.CreateDefaultContext())
        body.SetMass(plant_context, mass+pair[1])


def DrakeCartPoleEnv(
        meshcat=None,
        time_limit=gym_time_limit,
        debug=False,
        obs_noise=False,
        monitoring_camera=False,
        add_disturbances=False):

    # Make simulation.
    simulator = make_sim(meshcat=meshcat,
                         time_limit=time_limit,
                         debug=debug,
                         obs_noise=obs_noise,
                         monitoring_camera=monitoring_camera,
                         add_disturbances=add_disturbances)

    plant = simulator.get_system().GetSubsystemByName("plant")

    # Define Action space.
    na = 1
    low_a = plant.GetEffortLowerLimits()[:na]
    high_a = plant.GetEffortUpperLimits()[:na]
    action_space = gym.spaces.Box(low=np.asarray(low_a, dtype="float32"),
                                  high=np.asarray(high_a, dtype="float32"),
                                  dtype=np.float32)

    # Define observation space.
    low = np.concatenate(
        (plant.GetPositionLowerLimits(), plant.GetVelocityLowerLimits()))
    high = np.concatenate(
        (plant.GetPositionUpperLimits(), plant.GetVelocityUpperLimits()))
    observation_space = gym.spaces.Box(low=np.asarray(low),
                                       high=np.asarray(high),
                                       dtype=np.float64)

    env = DrakeGymEnv(
        simulator=simulator,
        time_step=gym_time_step,
        action_space=action_space,
        observation_space=observation_space,
        reward="reward",
        action_port_id="actions",
        observation_port_id="observations",
        reset_handler=reset_handler,
        render_rgb_port_id="color_image" if monitoring_camera else None)

    # Expose parameters that could be useful for learning.
    env.time_step = gym_time_step
    env.sim_time_step = sim_time_step

    return env
