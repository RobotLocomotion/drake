"""
Runs manipulation_station example with Sony DualShock4 Controller for
teleoperating the end effector.
"""

import argparse
from enum import Enum
import os
import pprint
import sys
from textwrap import dedent

import numpy as np

from pydrake.examples.manipulation_station import (
    ManipulationStation, ManipulationStationHardwareInterface,
    CreateClutterClearingYcbObjectList, SchunkCollisionModel)
from pydrake.geometry import DrakeVisualizer
from pydrake.multibody.plant import MultibodyPlant
from pydrake.manipulation.planner import (
    DifferentialInverseKinematicsParameters)
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem
from pydrake.systems.meshcat_visualizer import (
    ConnectMeshcatVisualizer, MeshcatVisualizer)
from pydrake.systems.primitives import FirstOrderLowPassFilter

from drake.examples.manipulation_station.differential_ik import DifferentialIK

# On macOS, our setup scripts do not provide pygame so we need to skip this
# program and its tests.  On Ubuntu, we do expect to have pygame.
if sys.platform == "darwin":
    try:
        import pygame
        from pygame.locals import *
    except ImportError:
        print("ERROR: missing pygame.  "
              "Please install pygame to use this example.")
        sys.exit(0)
else:
    import pygame
    from pygame.locals import *


def initialize_joystick(joystick_id):
    assert isinstance(joystick_id, (int, type(None)))
    pygame.init()
    try:
        pygame.joystick.init()
        if joystick_id is None:
            count = pygame.joystick.get_count()
            if count != 1:
                raise RuntimeError(
                    f"joystick_id=None, but there are {count} joysticks "
                    f"plugged in. Please specify --joystick_id, or ensure "
                    f"that exactly 1 joystick is plugged in")
            joystick_id = 0
        joystick = pygame.joystick.Joystick(joystick_id)
        joystick.init()
        return joystick
    except pygame.error as e:
        raise Exception("Make sure dualshock 4 controller is connected. "
                        "Controller initialization failed "
                        f"with: {e}")


class DS4Buttons(Enum):
    X_BUTTON = 0
    O_BUTTON = 1
    TRIANGLE_BUTTON = 2
    SQUARE_BUTTON = 3
    L1_BUTTON = 4
    R1_BUTTON = 5
    L2_BUTTON = 6
    R2_BUTTON = 7


class DS4Axis(Enum):
    LEFTJOY_UP_DOWN = 0
    LEFTJOY_LEFT_RIGHT = 1
    RIGHTJOY_LEFT_RIGHT = 2
    RIGHTJOY_UP_DOWN = 3


def print_instructions():
    instructions = '''\

        END EFFECTOR CONTROL
        -----------------------------------------
        +/- x-axis         - leftjoy left / right
        +/- y-axis         - leftjoy up / down
        +/- roll           - rightjoy up / down
        +/- pitch          - rightjoy left / right
        +/- z-axis         - l2 / r2
        +/- yaw            - l1 / r1

        GRIPPER CONTROL
        -----------------------------------------
        open / close       - square / circle (O)

        -----------------------------------------
        x button           - quit
    '''
    print(dedent(instructions))


class TeleopDualShock4Manager:

    def __init__(self, joystick):
        self._joystick = joystick
        self._axis_data = list()
        self._button_data = list()
        self._name = joystick.get_name()
        print(f"Using Joystick: {self._name}")

        for i in range(self._joystick.get_numbuttons()):
            self._button_data.append(False)

        for i in range(self._joystick.get_numaxes()):
            self._axis_data.append(0.0)

    def get_events(self):
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                self._axis_data[event.axis] = round(event.value, 2)
            if event.type == pygame.JOYBUTTONDOWN:
                self._button_data[event.button] = True
            if event.type == pygame.JOYBUTTONUP:
                self._button_data[event.button] = False

        events = dict()
        # For example mappings, see:
        # https://www.pygame.org/docs/ref/joystick.html#controller-mappings
        if self._name == "Logitech Logitech Dual Action":
            events[DS4Axis.LEFTJOY_LEFT_RIGHT] = self._axis_data[0]
            events[DS4Axis.LEFTJOY_UP_DOWN] = self._axis_data[1]
            events[DS4Axis.RIGHTJOY_LEFT_RIGHT] = self._axis_data[2]
            events[DS4Axis.RIGHTJOY_UP_DOWN] = self._axis_data[3]
        else:
            events[DS4Axis.LEFTJOY_UP_DOWN] = self._axis_data[0]
            events[DS4Axis.LEFTJOY_LEFT_RIGHT] = self._axis_data[1]
            events[DS4Axis.RIGHTJOY_LEFT_RIGHT] = self._axis_data[3]
            events[DS4Axis.RIGHTJOY_UP_DOWN] = self._axis_data[4]
        events[DS4Buttons.X_BUTTON] = self._button_data[0]
        events[DS4Buttons.O_BUTTON] = self._button_data[1]
        events[DS4Buttons.SQUARE_BUTTON] = self._button_data[3]
        events[DS4Buttons.L1_BUTTON] = self._button_data[4]
        events[DS4Buttons.R1_BUTTON] = self._button_data[5]
        events[DS4Buttons.L2_BUTTON] = self._button_data[6]
        events[DS4Buttons.R2_BUTTON] = self._button_data[7]

        # TODO(eric.cousineau): Replace `sys.exit` with a status to
        # the Systems Framework.
        if events[DS4Buttons.X_BUTTON]:
            sys.exit(0)
        return events


class DualShock4Teleop(LeafSystem):
    def __init__(self, joystick):
        LeafSystem.__init__(self)
        self.DeclareVectorOutputPort("rpy_xyz", 6, self.DoCalcOutput)
        self.DeclareVectorOutputPort("position", 1, self.CalcPositionOutput)
        self.DeclareVectorOutputPort("force_limit", 1,
                                     self.CalcForceLimitOutput)

        # Note: This timing affects the keyboard teleop performance. A larger
        #       time step causes more lag in the response.
        self.DeclarePeriodicPublish(1.0, 0.0)

        self.teleop_manager = TeleopDualShock4Manager(joystick)
        self.roll = self.pitch = self.yaw = 0
        self.x = self.y = self.z = 0
        self.gripper_max = 0.107
        self.gripper_min = 0.01
        self.gripper_goal = self.gripper_max

    def SetPose(self, pose):
        """
        @param pose is a RigidTransform or else any type accepted by
                    RigidTransform's constructor
        """
        tf = RigidTransform(pose)
        self.SetRPY(RollPitchYaw(tf.rotation()))
        self.SetXYZ(pose.translation())

    def SetRPY(self, rpy):
        """
        @param rpy is a RollPitchYaw object
        """
        self.roll = rpy.roll_angle()
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
        scale = 0.00005

        delta_x = scale * events[DS4Axis.LEFTJOY_UP_DOWN]
        delta_y = -scale * events[DS4Axis.LEFTJOY_LEFT_RIGHT]
        delta_z = 0.0

        if events[DS4Buttons.L2_BUTTON]:
            delta_z += scale
        if events[DS4Buttons.R2_BUTTON]:
            delta_z -= scale

        self.x += delta_x
        self.y += delta_y
        self.z += delta_z

    def SetRpyFromEvents(self, events):
        roll_scale = 0.0001
        delta_roll = -roll_scale * events[DS4Axis.RIGHTJOY_LEFT_RIGHT]
        self.roll = np.clip(self.roll + delta_roll,
                            a_min=-2*np.pi, a_max=2*np.pi)

        pitch_scale = 0.0001
        delta_pitch = pitch_scale * events[DS4Axis.RIGHTJOY_UP_DOWN]
        self.pitch = np.clip(self.pitch + delta_pitch,
                             a_min=-2*np.pi, a_max=2*np.pi)

        yaw_scale = 0.0003
        if events[DS4Buttons.L1_BUTTON]:
            self.yaw += yaw_scale
        if events[DS4Buttons.R1_BUTTON]:
            self.yaw -= yaw_scale
        self.yaw = np.clip(self.yaw, a_min=-2*np.pi, a_max=2*np.pi)

    def SetGripperFromEvents(self, events):
        gripper_scale = 0.00005
        self.gripper_goal += (gripper_scale * events[DS4Buttons.SQUARE_BUTTON])
        self.gripper_goal -= (gripper_scale * events[DS4Buttons.O_BUTTON])
        self.gripper_goal = np.clip(self.gripper_goal,
                                    a_min=self.gripper_min,
                                    a_max=self.gripper_max)

    def CalcPositionOutput(self, context, output):
        output.SetAtIndex(0, self.gripper_goal)

    def CalcForceLimitOutput(self, context, output):
        self._force_limit = 40
        output.SetAtIndex(0, self._force_limit)

    def DoCalcOutput(self, context, output):
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


def main():
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
        "--joystick_id", type=int, default=None,
        help="Joystick ID to use (0..N-1). If not specified, then only one "
             "joystick must be plugged in, and that joystick will be used.")
    parser.add_argument(
        "--test", action='store_true',
        help="Disable opening the gui window for testing.")
    parser.add_argument(
        "--time_step", type=float, default=0.005,
        help="Time constant for the differential IK solver and first order"
             "low pass filter applied to the teleop commands")
    parser.add_argument(
        "--velocity_limit_factor", type=float, default=1.0,
        help="This value, typically between 0 and 1, further limits the "
             "iiwa14 joint velocities. It multiplies each of the seven "
             "pre-defined joint velocity limits. "
             "Note: The pre-defined velocity limits are specified by "
             "iiwa14_velocity_limits, found in this python file.")
    parser.add_argument(
        '--setup', type=str, default='manipulation_class',
        help="The manipulation station setup to simulate. ",
        choices=['manipulation_class', 'clutter_clearing'])
    parser.add_argument(
        '--schunk_collision_model', type=str, default='box',
        help="The Schunk collision model to use for simulation. ",
        choices=['box', 'box_plus_fingertip_spheres'])
    MeshcatVisualizer.add_argparse_argument(parser)
    args = parser.parse_args()

    if args.test:
        # Don't grab mouse focus during testing.
        # See: https://stackoverflow.com/a/52528832/7829525
        os.environ["SDL_VIDEODRIVER"] = "dummy"

    builder = DiagramBuilder()

    if args.hardware:
        station = builder.AddSystem(ManipulationStationHardwareInterface())
        station.Connect(wait_for_cameras=False)
    else:
        station = builder.AddSystem(ManipulationStation())

        if args.schunk_collision_model == "box":
            schunk_model = SchunkCollisionModel.kBox
        elif args.schunk_collision_model == "box_plus_fingertip_spheres":
            schunk_model = SchunkCollisionModel.kBoxPlusFingertipSpheres

    # Initializes the chosen station type.
        if args.setup == 'manipulation_class':
            station.SetupManipulationClassStation(
                schunk_model=schunk_model)
            station.AddManipulandFromFile(
                ("drake/examples/manipulation_station/models/"
                 "061_foam_brick.sdf"),
                RigidTransform(RotationMatrix.Identity(), [0.6, 0, 0]))
        elif args.setup == 'clutter_clearing':
            station.SetupClutterClearingStation(
                schunk_model=schunk_model)
            ycb_objects = CreateClutterClearingYcbObjectList()
            for model_file, X_WObject in ycb_objects:
                station.AddManipulandFromFile(model_file, X_WObject)

        station.Finalize()
        DrakeVisualizer.AddToBuilder(builder,
                                     station.GetOutputPort("query_object"))
        if args.meshcat:
            meshcat = ConnectMeshcatVisualizer(
                builder, output_port=station.GetOutputPort("geometry_query"),
                zmq_url=args.meshcat, open_browser=args.open_browser)
            if args.setup == 'planar':
                meshcat.set_planar_viewpoint()

    robot = station.get_controller_plant()
    params = DifferentialInverseKinematicsParameters(robot.num_positions(),
                                                     robot.num_velocities())

    params.set_timestep(args.time_step)
    # True velocity limits for the IIWA14 (in rad/s, rounded down to the first
    # decimal)
    iiwa14_velocity_limits = np.array([1.4, 1.4, 1.7, 1.3, 2.2, 2.3, 2.3])
    # Stay within a small fraction of those limits for this teleop demo.
    factor = args.velocity_limit_factor
    params.set_joint_velocity_limits((-factor*iiwa14_velocity_limits,
                                      factor*iiwa14_velocity_limits))

    differential_ik = builder.AddSystem(DifferentialIK(
        robot, robot.GetFrameByName("iiwa_link_7"), params, args.time_step))

    builder.Connect(differential_ik.GetOutputPort("joint_position_desired"),
                    station.GetInputPort("iiwa_position"))

    joystick = initialize_joystick(args.joystick_id)
    teleop = builder.AddSystem(DualShock4Teleop(joystick))
    filter_ = builder.AddSystem(
        FirstOrderLowPassFilter(time_constant=args.time_step, size=6))

    builder.Connect(teleop.get_output_port(0), filter_.get_input_port(0))
    builder.Connect(filter_.get_output_port(0),
                    differential_ik.GetInputPort("rpy_xyz_desired"))

    builder.Connect(teleop.GetOutputPort("position"), station.GetInputPort(
        "wsg_position"))
    builder.Connect(teleop.GetOutputPort("force_limit"),
                    station.GetInputPort("wsg_force_limit"))

    diagram = builder.Build()
    simulator = Simulator(diagram)

    # This is important to avoid duplicate publishes to the hardware interface:
    simulator.set_publish_every_time_step(False)

    station_context = diagram.GetMutableSubsystemContext(
        station, simulator.get_mutable_context())

    station.GetInputPort("iiwa_feedforward_torque").FixValue(
        station_context, np.zeros(7))

    # If the diagram is only the hardware interface, then we must advance it a
    # little bit so that first LCM messages get processed. A simulated plant is
    # already publishing correct positions even without advancing, and indeed
    # we must not advance a simulated plant until the sliders and filters have
    # been initialized to match the plant.
    if args.hardware:
        simulator.AdvanceTo(1e-6)

    q0 = station.GetOutputPort("iiwa_position_measured").Eval(station_context)
    differential_ik.parameters.set_nominal_joint_position(q0)

    teleop.SetPose(differential_ik.ForwardKinematics(q0))
    filter_.set_initial_output_value(
        diagram.GetMutableSubsystemContext(
            filter_, simulator.get_mutable_context()),
        teleop.get_output_port(0).Eval(diagram.GetMutableSubsystemContext(
            teleop, simulator.get_mutable_context())))
    differential_ik.SetPositions(diagram.GetMutableSubsystemContext(
        differential_ik, simulator.get_mutable_context()), q0)

    simulator.set_target_realtime_rate(args.target_realtime_rate)

    print_instructions()
    simulator.AdvanceTo(args.duration)


if __name__ == '__main__':
    main()
