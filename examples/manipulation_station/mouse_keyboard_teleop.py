import argparse
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.examples.manipulation_station import (
    ManipulationStation, ManipulationStationHardwareInterface)
from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.manipulation.planner import (
    DifferentialInverseKinematicsParameters, DoDifferentialInverseKinematics)
from pydrake.multibody.parsing import Parser
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import (AbstractValue, BasicVector,
                                       DiagramBuilder, LeafSystem,
                                       PortDataType)
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.systems.primitives import FirstOrderLowPassFilter
from pydrake.util.eigen_geometry import Isometry3, AngleAxis

from differential_ik import DifferentialIK
import sys
import pygame
from pygame.locals import *


def print_instructions():
    print("")
    print("END EFFECTOR CONTROL")
    print("mouse left/right   - move in the manipulation station's y/z plane")
    print("w / s              - move forward/back this y/z plane")
    print("mouse buttons      - roll left/right")
    print("side mouse buttons - yaw left/right")
    print("a / d              - pitch up/down")
    print("")
    print("")
    print("GRIPPER CONTROL")
    print("mouse wheel        - open/close gripper")
    print("")
    print("escape             - quit")

class TeleopMouseKeyboardManager():

    def __init__(self):

        pygame.init()
        # We don't actually want a screen, but I can't get this to work without a tiny screen.
        # Setting it to 1 pixel
        screen_size = 1 
        self.screen = pygame.display.set_mode((screen_size, screen_size))

        # These will "grab focus" of the mouse
        pygame.event.set_grab(True)
        pygame.mouse.set_visible(False)

        self.side_button_back_DOWN = False
        self.side_button_fwd_DOWN = False

     
    def get_events(self):
        mouse_wheel_up = mouse_wheel_down = False

        for event in pygame.event.get():
            if event.type == QUIT:
                sys.exit(0)
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                sys.exit(0)
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:   
                    mouse_wheel_up = True
                if event.button == 5:   
                    mouse_wheel_down = True
                if event.button == 8:
                    self.side_button_back_DOWN = True
                if event.button == 9:
                    self.side_button_fwd_DOWN = True
            if event.type == pygame.MOUSEBUTTONUP:
                if event.button == 8:
                    self.side_button_back_DOWN = False
                if event.button == 9:
                    self.side_button_fwd_DOWN = False

        keys = pygame.key.get_pressed()
        delta_x, delta_y = pygame.mouse.get_rel()
        left_mouse_button, _, right_mouse_button = pygame.mouse.get_pressed()

        events = dict()
        events["delta_x"] = delta_x
        events["delta_y"] = delta_y
        events["w"] = keys[K_w]
        events["a"] = keys[K_a]
        events["s"] = keys[K_s]
        events["d"] = keys[K_d]
        events["r"] = keys[K_r]
        events["mouse_wheel_up"] = mouse_wheel_up
        events["mouse_wheel_down"] = mouse_wheel_down
        events["left_mouse_button"] = left_mouse_button
        events["right_mouse_button"] = right_mouse_button
        events["side_button_back"] = self.side_button_back_DOWN
        events["side_button_forward"] = self.side_button_fwd_DOWN
        return events 


class MouseKeyboardTeleop(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self._DeclareVectorOutputPort("rpy_xyz", BasicVector(6),
                                      self._DoCalcOutput)
        self._DeclareVectorOutputPort("position", BasicVector(1),
                                      self.CalcPositionOutput)
        self._DeclareVectorOutputPort("force_limit", BasicVector(1),
                                      self.CalcForceLimitOutput)

        # Note: This timing affects the keyboard teleop performance. A larger
        #       time step causes more lag in the response.
        self._DeclarePeriodicPublish(0.01, 0.0)

        self.teleop_manager = TeleopMouseKeyboardManager()
        self.roll = self.pitch = self.yaw = 0
        self.x = self.y = self.z = 0
        self.gripper_max = 0.107
        self.gripper_min = 0.02
        self.gripper_goal = self.gripper_max

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
        self.roll= rpy.roll_angle()
        self.pitch = rpy.pitch_angle()
        self.yaw = rpy.yaw_angle()

    def SetXYZ(self, xyz):
        """
        @param xyz is a 3 element vector of x, y, z.
        """
        self.x = xyz[0]
        self.y = xyz[1]
        self.z = xyz[2]

    def SetXyzFromEvents(self, events):
        scale_down = 0.0001
        delta_x = events["delta_x"]*-scale_down
        delta_y = events["delta_y"]*-scale_down
        
        forward_scale = 0.00005
        delta_forward = 0.0
        if events["w"]:
            delta_forward += forward_scale
        if events["s"]:
            delta_forward -= forward_scale

        self.x += delta_forward
        self.y += delta_x
        self.z += delta_y

    def SetRpyFromEvents(self, events):
        roll_scale = 0.0003
        if events["left_mouse_button"]:
            self.roll += roll_scale
        if events["right_mouse_button"]:
            self.roll -= roll_scale
        self.roll = np.clip(self.roll, a_min = -2 * np.pi, a_max = 2 * np.pi)
        
        yaw_scale = 0.0003
        if events["side_button_back"]:
            self.yaw += yaw_scale
        if events["side_button_forward"]:
            self.yaw -= yaw_scale
        self.yaw = np.clip(self.yaw, a_min = -2 * np.pi, a_max = 2 * np.pi)

        pitch_scale = 0.0003
        if events["d"]:
            self.pitch += pitch_scale
        if events["a"]:
            self.pitch -= pitch_scale
        self.pitch = np.clip(self.pitch, a_min = -2 * np.pi, a_max = 2 * np.pi)

    def SetGripperFromEvents(self, events):
        gripper_scale = 0.01
        if events["mouse_wheel_up"]:
            self.gripper_goal += gripper_scale
        if events["mouse_wheel_down"]:
            self.gripper_goal -= gripper_scale
        self.gripper_goal = np.clip(self.gripper_goal, a_min = self.gripper_min, a_max = self.gripper_max) 

    def CalcPositionOutput(self, context, output):
        output.SetAtIndex(0, self.gripper_goal)

    def CalcForceLimitOutput(self, context, output):
        self._force_limit = 40
        output.SetAtIndex(0, self._force_limit)

    def _DoCalcOutput(self, context, output):
        events = self.teleop_manager.get_events()
        self.SetXyzFromEvents(events)
        self.SetRpyFromEvents(events)
        self.SetGripperFromEvents(events)
        output.SetAtIndex(0, self.roll)
        output.SetAtIndex(1, self.pitch)
        output.SetAtIndex(2, self.yaw)
        output.SetAtIndex(3, self.x)
        output.SetAtIndex(4, self.y)
        output.SetAtIndex(5, self.z)


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
parser.add_argument(
    "--filter_time_const", type=float, default=0.005,
    help="Time constant for the first order low pass filter applied to"
         "the teleop commands")
parser.add_argument(
    "--velocity_limit_factor", type=float, default=0.4,
    help="This value, typically between 0 and 1, further limits the iiwa14 "
         "joint velocities. It multiplies each of the seven pre-defined "
         "joint velocity limits. "
         "Note: The pre-defined velocity limits are specified by "
         "iiwa14_velocity_limits, found in this python file.")
parser.add_argument(
    '--setup', type=str, default='default',
    help="The manipulation station setup to simulate. ",
    choices=['default', 'clutter_clearing'])

MeshcatVisualizer.add_argparse_argument(parser)
args = parser.parse_args()

builder = DiagramBuilder()

if args.hardware:
    station = builder.AddSystem(ManipulationStationHardwareInterface())
    station.Connect(wait_for_cameras=False)
else:
    station = builder.AddSystem(ManipulationStation())

    # Initializes the chosen station type.
    if args.setup == 'default':
        station.SetupDefaultStation()
    elif args.setup == 'clutter_clearing':
        station.SetupClutterClearingStation()

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
factor = args.velocity_limit_factor
params.set_joint_velocity_limits((-factor*iiwa14_velocity_limits,
                                  factor*iiwa14_velocity_limits))

differential_ik = builder.AddSystem(DifferentialIK(
    robot, robot.GetFrameByName("iiwa_link_7"), params, time_step))

builder.Connect(differential_ik.GetOutputPort("joint_position_desired"),
                station.GetInputPort("iiwa_position"))

teleop = builder.AddSystem(MouseKeyboardTeleop())
if args.test:
    teleop.window.withdraw()  # Don't display the window when testing.
filter = builder.AddSystem(
    FirstOrderLowPassFilter(time_constant=args.filter_time_const, size=6))

builder.Connect(teleop.get_output_port(0), filter.get_input_port(0))
builder.Connect(filter.get_output_port(0),
                differential_ik.GetInputPort("rpy_xyz_desired"))

builder.Connect(teleop.GetOutputPort("position"), station.GetInputPort(
    "wsg_position"))
builder.Connect(teleop.GetOutputPort("force_limit"),
                station.GetInputPort("wsg_force_limit"))

diagram = builder.Build()
simulator = Simulator(diagram)

station_context = diagram.GetMutableSubsystemContext(
    station, simulator.get_mutable_context())

station_context.FixInputPort(station.GetInputPort(
    "iiwa_feedforward_torque").get_index(), np.zeros(7))

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

print_instructions()
simulator.StepTo(args.duration)
