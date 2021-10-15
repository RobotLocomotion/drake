import argparse
import os
import sys

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


def print_instructions():
    print("")
    print("END EFFECTOR CONTROL")
    print("mouse left/right   - move in the manipulation station's y/z plane")
    print("mouse buttons      - roll left/right")
    print("w / s              - move forward/back this y/z plane")
    print("q / e              - yaw left/right \
                                (also can use mouse side buttons)")
    print("a / d              - pitch up/down")
    print("")
    print("GRIPPER CONTROL")
    print("mouse wheel        - open/close gripper")
    print("")
    print("space              - switch out of teleop mode")
    print("enter              - return to teleop mode (be sure you've")
    print("                     returned focus to the pygame app)")
    print("escape             - quit")


class TeleopMouseKeyboardManager():

    def __init__(self, grab_focus=True):
        pygame.init()
        # We don't actually want a screen, but
        # I can't get this to work without a tiny screen.
        # Setting it to 1 pixel.
        screen_size = 1
        self.screen = pygame.display.set_mode((screen_size, screen_size))

        self.side_button_back_DOWN = False
        self.side_button_fwd_DOWN = False
        if grab_focus:
            self.grab_mouse_focus()

    def grab_mouse_focus(self):
        pygame.event.set_grab(True)
        pygame.mouse.set_visible(False)

    def release_mouse_focus(self):
        pygame.event.set_grab(False)
        pygame.mouse.set_visible(True)

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

        if keys[K_RETURN]:
            self.grab_mouse_focus()
        if keys[K_SPACE]:
            self.release_mouse_focus()

        events = dict()
        events["delta_x"] = delta_x
        events["delta_y"] = delta_y
        events["w"] = keys[K_w]
        events["a"] = keys[K_a]
        events["s"] = keys[K_s]
        events["d"] = keys[K_d]
        events["r"] = keys[K_r]
        events["q"] = keys[K_q]
        events["e"] = keys[K_e]
        events["mouse_wheel_up"] = mouse_wheel_up
        events["mouse_wheel_down"] = mouse_wheel_down
        events["left_mouse_button"] = left_mouse_button
        events["right_mouse_button"] = right_mouse_button
        events["side_button_back"] = self.side_button_back_DOWN
        events["side_button_forward"] = self.side_button_fwd_DOWN
        return events


class MouseKeyboardTeleop(LeafSystem):
    def __init__(self, grab_focus=True):
        LeafSystem.__init__(self)
        self.DeclareVectorOutputPort("rpy_xyz", 6, self.DoCalcOutput)
        self.DeclareVectorOutputPort("position", 1, self.CalcPositionOutput)
        self.DeclareVectorOutputPort("force_limit", 1,
                                     self.CalcForceLimitOutput)

        # Note: This timing affects the keyboard teleop performance. A larger
        #       time step causes more lag in the response.
        self.DeclarePeriodicPublish(0.01, 0.0)

        self.teleop_manager = TeleopMouseKeyboardManager(grab_focus=grab_focus)
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
        self.roll = np.clip(self.roll, a_min=-2*np.pi, a_max=2*np.pi)

        yaw_scale = 0.0003
        if events["side_button_back"] or events["q"]:
            self.yaw += yaw_scale
        if events["side_button_forward"] or events["e"]:
            self.yaw -= yaw_scale
        self.yaw = np.clip(self.yaw, a_min=-2*np.pi, a_max=2*np.pi)

        pitch_scale = 0.0003
        if events["d"]:
            self.pitch += pitch_scale
        if events["a"]:
            self.pitch -= pitch_scale
        self.pitch = np.clip(self.pitch, a_min=-2*np.pi, a_max=2*np.pi)

    def SetGripperFromEvents(self, events):
        gripper_scale = 0.01
        if events["mouse_wheel_up"]:
            self.gripper_goal += gripper_scale
        if events["mouse_wheel_down"]:
            self.gripper_goal -= gripper_scale
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
        "--test", action='store_true',
        help="Disable opening the gui window for testing.")
    parser.add_argument(
        "--filter_time_const", type=float, default=0.005,
        help="Time constant for the first order low pass filter applied to"
             "the teleop commands")
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
        grab_focus = False
        # See: https://stackoverflow.com/a/52528832/7829525
        os.environ["SDL_VIDEODRIVER"] = "dummy"
    else:
        grab_focus = True

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

    teleop = builder.AddSystem(MouseKeyboardTeleop(grab_focus=grab_focus))
    filter_ = builder.AddSystem(
        FirstOrderLowPassFilter(time_constant=args.filter_time_const, size=6))

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
