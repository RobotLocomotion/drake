
import argparse

import Tkinter as tk
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.examples.manipulation_station import (
    ManipulationStation, ManipulationStationHardwareInterface)
from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.manipulation.simple_ui import SchunkWsgButtons
from pydrake.manipulation.planner import (
    DifferentialInverseKinematicsParameters, DoDifferentialInverseKinematics)
from pydrake.multibody.parsing import Parser
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import (AbstractValue, BasicVector,
                                       DiagramBuilder, LeafSystem,
                                       PortDataType)
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.systems.primitives import FirstOrderLowPassFilter
from pydrake.util.eigen_geometry import Isometry3, AngleAxis


# TODO(russt): Generalize this and move it to pydrake.manipulation.simple_ui.
class EndEffectorTeleop(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self._DeclareVectorOutputPort("rpy_xyz", BasicVector(6),
                                      self._DoCalcOutput)

        self._DeclarePeriodicPublish(0.1, 0.0)

        self.window = tk.Tk()
        self.window.title("End-Effector TeleOp")

        self.roll = tk.Scale(self.window, from_=-2 * np.pi, to=2 * np.pi,
                             resolution=-1,
                             label="roll",
                             length=800,
                             orient=tk.HORIZONTAL)
        self.roll.pack()
        self.pitch = tk.Scale(self.window, from_=-2 * np.pi, to=2 * np.pi,
                              resolution=-1,
                              label="pitch",
                              length=800,
                              orient=tk.HORIZONTAL)
        self.pitch.pack()
        self.yaw = tk.Scale(self.window, from_=-2 * np.pi, to=2 * np.pi,
                            resolution=-1,
                            label="yaw",
                            length=800,
                            orient=tk.HORIZONTAL)
        self.yaw.pack()
        self.x = tk.Scale(self.window, from_=0.1, to=0.8,
                          resolution=-1,
                          label="x",
                          length=800,
                          orient=tk.HORIZONTAL)
        self.x.pack()
        self.y = tk.Scale(self.window, from_=-0.3, to=0.3,
                          resolution=-1,
                          label="y",
                          length=800,
                          orient=tk.HORIZONTAL)
        self.y.pack()
        self.z = tk.Scale(self.window, from_=0, to=0.8,
                          resolution=-1,
                          label="z",
                          length=800,
                          orient=tk.HORIZONTAL)
        self.z.pack()

    def SetPose(self, pose):
        """
        @param pose is an Isometry3.
        """
        tf = RigidTransform(pose)
        self.SetRPY(RollPitchYaw(tf.rotation()))
        self.SetXYZ(pose.translation())

    def SetRPY(self, rpy):
        """
        @param rpy is a 3 element vector of roll, pitch, yaw.
        """
        self.roll.set(rpy.roll_angle())
        self.pitch.set(rpy.pitch_angle())
        self.yaw.set(rpy.yaw_angle())

    def SetXYZ(self, xyz):
        """
        @param xyz is a 3 element vector of x, y, z.
        """
        self.x.set(xyz[0])
        self.y.set(xyz[1])
        self.z.set(xyz[2])

    def _DoPublish(self, context, event):
        self.window.update_idletasks()
        self.window.update()

    def _DoCalcOutput(self, context, output):
        output.SetAtIndex(0, self.roll.get())
        output.SetAtIndex(1, self.pitch.get())
        output.SetAtIndex(2, self.yaw.get())
        output.SetAtIndex(3, self.x.get())
        output.SetAtIndex(4, self.y.get())
        output.SetAtIndex(5, self.z.get())


# TODO(russt): Clean this up and move it to C++.
class DifferentialIK(LeafSystem):
    """
    A simple system that wraps calls to the DifferentialInverseKinematics API.
    It is highly recommended that the user calls SetPosition() once to
    initialize the initial position commands to match the initial
    configuration of the robot.

    @system{
      @input_port{X_WE_desired},
      @output_port{joint_position_desired}
    """
    def __init__(self, robot, frame_E, parameters, time_step):
        """
        @param robot is a reference to a MultibodyPlant.
        @param frame_E is a multibody::Frame on the robot.
        @param params is a DifferentialIKParams.
        @params time_step This system updates its state/outputs at discrete
                          periodic intervals defined with period @p time_step.
        """
        LeafSystem.__init__(self)
        self.robot = robot
        self.frame_E = frame_E
        self.parameters = parameters
        self.parameters.set_timestep(time_step)
        self.time_step = time_step
        # Note that this context is NOT the context of the DifferentialIK
        # system, but rather a context for the multibody plant that is used
        # to pass the configuation into the DifferentialInverseKinematics
        # methods.
        self.robot_context = robot.CreateDefaultContext()
        # Confirm that all velocities are zero (they will not be reset below).
        assert not self.robot.tree().GetPositionsAndVelocities(
            self.robot_context)[-robot.num_velocities():].any()

        # Store the robot positions as state.
        self._DeclareDiscreteState(robot.num_positions())
        self._DeclarePeriodicDiscreteUpdate(time_step)

        # Desired pose of frame E in world frame.
        self._DeclareInputPort("rpy_xyz_desired",
                               PortDataType.kVectorValued, 6)

        # Provide the output as desired positions.
        self._DeclareVectorOutputPort("joint_position_desired", BasicVector(
            robot.num_positions()), self.CopyPositionOut)

    def SetPositions(self, context, q):
        context.get_mutable_discrete_state(0).SetFromVector(q)

    def ForwardKinematics(self, q):
        x = self.robot.tree().GetMutablePositionsAndVelocities(
            self.robot_context)
        x[:robot.num_positions()] = q
        return self.robot.tree().EvalBodyPoseInWorld(
            self.robot_context, self.frame_E.body())

    def CalcPoseError(self, X_WE_desired, q):
        pose = self.ForwardKinematics(q)
        err_vec = np.zeros(6)
        err_vec[-3:] = X_WE_desired.translation() - pose.translation()

        rot_err = AngleAxis(X_WE_desired.rotation() *
                            pose.rotation().transpose())
        err_vec[:3] = rot_err.axis() * rot_err.angle()

    def _DoCalcDiscreteVariableUpdates(
            self, context, events, discrete_state):
        rpy_xyz_desired = self.EvalVectorInput(context, 0).get_value()
        X_WE_desired = RigidTransform(RollPitchYaw(rpy_xyz_desired[:3]),
                                      rpy_xyz_desired[-3:]).GetAsIsometry3()
        q_last = context.get_discrete_state_vector().get_value()

        x = self.robot.tree().GetMutablePositionsAndVelocities(
            self.robot_context)
        x[:robot.num_positions()] = q_last
        result = DoDifferentialInverseKinematics(self.robot,
                                                 self.robot_context,
                                                 X_WE_desired, self.frame_E,
                                                 self.parameters)

        if (result.status != result.status.kSolutionFound):
            print("Differential IK could not find a solution.")
            discrete_state.get_mutable_vector().SetFromVector(q_last)
        else:
            discrete_state.get_mutable_vector().\
                SetFromVector(q_last + self.time_step*result.joint_velocities)

    def CopyPositionOut(self, context, output):
        output.SetFromVector(context.get_discrete_state_vector().get_value())


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--target_realtime_rate", type=float, default=1.0,
    help="Desired rate relative to real time.  See documentation for "
         "Simulator::set_target_realtime_rate() for details.")
parser.add_argument(
    "--duration", type=float, default=np.inf,
    help="Desired duration of the simulation in seconds.")
parser.add_argument(
    "--hardware", action='store_true',
    help="Use the ManipulationStationHardwareInterface instead of an "
         "in-process simulation.")
parser.add_argument(
    "--test", action='store_true',
    help="Disable opening the gui window for testing.")
MeshcatVisualizer.add_argparse_argument(parser)
args = parser.parse_args()

builder = DiagramBuilder()

if args.hardware:
    station = builder.AddSystem(ManipulationStationHardwareInterface())
    station.Connect(wait_for_cameras=False)
else:
    station = builder.AddSystem(ManipulationStation())
    station.SetupDefaultStation()
    parser = Parser(station.get_mutable_multibody_plant(),
                    station.get_mutable_scene_graph())
    object = parser.AddModelFromFile(FindResourceOrThrow(
        "drake/examples/manipulation_station/models/061_foam_brick.sdf"),
        "object")
    station.Finalize()

    ConnectDrakeVisualizer(builder, station.get_scene_graph(),
                           station.GetOutputPort("pose_bundle"))
    if args.meshcat:
        meshcat = builder.AddSystem(MeshcatVisualizer(
            station.get_scene_graph(), zmq_url=args.meshcat))
        builder.Connect(station.GetOutputPort("pose_bundle"),
                        meshcat.get_input_port(0))

robot = station.get_controller_plant()
params = DifferentialInverseKinematicsParameters(robot.num_positions(),
                                                 robot.num_velocities())

time_step = 0.005
params.set_timestep(time_step)
# True velocity limits for the IIWA14 (in rad, rounded down to the first
# decimal)
iiwa14_velocity_limits = np.array([1.4, 1.4, 1.7, 1.3, 2.2, 2.3, 2.3])
# Stay within a small fraction of those limits for this teleop demo.
params.set_joint_velocity_limits((-.15*iiwa14_velocity_limits,
                                  .15*iiwa14_velocity_limits))

differential_ik = builder.AddSystem(DifferentialIK(
    robot, robot.GetFrameByName("iiwa_link_7"), params, time_step))

builder.Connect(differential_ik.GetOutputPort("joint_position_desired"),
                station.GetInputPort("iiwa_position"))

teleop = builder.AddSystem(EndEffectorTeleop())
if args.test:
    teleop.window.withdraw()  # Don't display the window when testing.
filter = builder.AddSystem(FirstOrderLowPassFilter(time_constant=2.0,
                                                   size=6))

builder.Connect(teleop.get_output_port(0), filter.get_input_port(0))
builder.Connect(filter.get_output_port(0),
                differential_ik.GetInputPort("rpy_xyz_desired"))

wsg_buttons = builder.AddSystem(SchunkWsgButtons(teleop.window))
builder.Connect(wsg_buttons.GetOutputPort("position"), station.GetInputPort(
    "wsg_position"))
builder.Connect(wsg_buttons.GetOutputPort("force_limit"),
                station.GetInputPort("wsg_force_limit"))

diagram = builder.Build()
simulator = Simulator(diagram)

station_context = diagram.GetMutableSubsystemContext(
    station, simulator.get_mutable_context())

station_context.FixInputPort(station.GetInputPort(
    "iiwa_feedforward_torque").get_index(), np.zeros(7))

if not args.hardware:
    # Set the initial positions of the IIWA to a comfortable configuration
    # inside the workspace of the station.
    q0 = [0, 0.6, 0, -1.75, 0, 1.0, 0]
    station.SetIiwaPosition(q0, station_context)
    station.SetIiwaVelocity(np.zeros(7), station_context)

    # Set the initial configuration of the gripper to open.
    station.SetWsgPosition(0.1, station_context)
    station.SetWsgVelocity(0, station_context)

    # Place the object in the middle of the workspace.
    X_WObject = Isometry3.Identity()
    X_WObject.set_translation([.6, 0, 0])
    station.get_multibody_plant().tree().SetFreeBodyPoseOrThrow(
        station.get_multibody_plant().GetBodyByName("base_link",
                                                    object),
        X_WObject, station.GetMutableSubsystemContext(
            station.get_multibody_plant(),
            station_context))

q0 = station.GetOutputPort("iiwa_position_measured").Eval(
    station_context).get_value()
differential_ik.parameters.set_nominal_joint_position(q0)

teleop.SetPose(differential_ik.ForwardKinematics(q0))
filter.set_initial_output_value(
    diagram.GetMutableSubsystemContext(
        filter, simulator.get_mutable_context()),
    teleop.get_output_port(0).Eval(diagram.GetMutableSubsystemContext(
        teleop, simulator.get_mutable_context())).get_value())
differential_ik.SetPositions(diagram.GetMutableSubsystemContext(
    differential_ik, simulator.get_mutable_context()), q0)

# This is important to avoid duplicate publishes to the hardware interface:
simulator.set_publish_every_time_step(False)

simulator.set_target_realtime_rate(args.target_realtime_rate)
simulator.StepTo(args.duration)
