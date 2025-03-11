import argparse
from pydrake.all import *
import numpy as np

##
#
# Simulate a franka arm with a controller that includes both high-gain feedback
# and effort limits.
#
##


class MyHighGainController(LeafSystem):
    """A high-gain PD controller with effort limits."""

    def __init__(self, kp, kd, nx, nu, u_max, x_nom):
        """Construct the custom controller.

        Args:
            kp: The proportional gain.
            kd: The derivative gain.
            nx: The number of states.
            nu: The number of control inputs.
            u_max: The maximum control inputs.
            x_nom: The nominal state.
        """
        super().__init__()
        self.state_input_port = self.DeclareVectorInputPort(
            name="state", size=nx
        )
        self.DeclareVectorOutputPort(
            name="control", size=nu, calc=self.CalcOutput
        )

        self.kp = kp
        self.kd = kd
        self.x_nom = x_nom
        self.u_max = u_max
        self.nq = nx // 2

    def CalcOutput(self, context, output):
        """Compute the control signal and set it in the output"""
        x = self.state_input_port.Eval(context)
        q = x[:self.nq]
        v = x[self.nq:]
        q_nom = self.x_nom[:self.nq]
        v_nom = self.x_nom[self.nq:]

        # PD controller
        u = self.kp * (q_nom - q) + self.kd * (v_nom - v)

        # Effort limits
        u = np.clip(u, -self.u_max, self.u_max)

        output.SetFromVector(u)


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
        help="Simulation time. Default: 10 seconds.",
    )
    parser.add_argument(
        "--mbp_time_step",
        type=float,
        default=0.0,
        help="Multibody plant time step. Default: 0.0.",
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=1e6,
        help="Proportional gain. Default: 1000.0.",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=1e4,
        help="Derivative gain. Default: 100.0.",
    )
    args = parser.parse_args()

    meshcat = StartMeshcat()
    builder = DiagramBuilder()

    # Create the plant model (franka panda arm)
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=args.mbp_time_step
    )
    Parser(plant).AddModelsFromUrl(
        "package://drake_models/franka_description/urdf/panda_arm_hand.urdf"
    )
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"))

    if plant.time_step() > 0:
        # Set up implicit PD controller for discrete SAP
        for idx in plant.GetJointActuatorIndices():
            actuator = plant.get_mutable_joint_actuator(idx)
            actuator.set_controller_gains(
                PdControllerGains(p=args.kp, d=args.kd)
            )

    plant.Finalize()

    # Set up the controller
    q_nom = np.array(
        [
            0.0,
            -0.4,
            0.5,
            -np.pi / 2,
            00,
            np.pi / 2,
            np.pi / 4,
            0.01,
            0.01,
        ]
    )
    v_nom = np.zeros(9)
    x_nom = np.concatenate([q_nom, v_nom])

    if plant.time_step() > 0:
        # For discrete SAP, we'll just connect a nominal state source
        nominal_state = builder.AddSystem(ConstantVectorSource(x_nom))
        builder.Connect(
            nominal_state.get_output_port(),
            plant.get_desired_state_input_port(
                plant.GetModelInstanceByName("panda")
            ),
        )
    else:
        # For continuous systems, we'll use the custom controller
        u_max = np.inf * np.ones(plant.num_actuators())
        for idx in plant.GetJointActuatorIndices():
            u_max[idx] = plant.get_joint_actuator(idx).effort_limit()

        ctrl = builder.AddSystem(
            MyHighGainController(
                kp=args.kp,
                kd=args.kd,
                nx=plant.num_positions() + plant.num_velocities(),
                nu=plant.num_actuators(),
                u_max=u_max,
                x_nom=x_nom,
            )
        )
        builder.Connect(
            ctrl.get_output_port(), plant.get_actuation_input_port()
        )
        builder.Connect(plant.get_state_output_port(), ctrl.get_input_port())

    # Finalize the diagram
    AddDefaultVisualization(builder=builder, meshcat=meshcat)
    diagram = builder.Build()

    # Set the initial state
    context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, context)

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
    simulator.AdvanceTo(args.sim_time)
    meshcat.StopRecording()
    meshcat.PublishRecording()

    # Print a summary of solver statistics
    PrintSimulatorStatistics(simulator)

    input("Waiting for meshcat... press [ENTER] to continue")
