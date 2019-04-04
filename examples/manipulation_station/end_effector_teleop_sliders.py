
import argparse

try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk
import numpy as np

from pydrake.examples.manipulation_station import (
    ManipulationStation, ManipulationStationHardwareInterface,
    CreateDefaultYcbObjectList)
from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.multibody.plant import MultibodyPlant
from pydrake.manipulation.simple_ui import SchunkWsgButtons
from pydrake.manipulation.planner import (
    DifferentialInverseKinematicsParameters)
from pydrake.multibody.parsing import Parser
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import (BasicVector, DiagramBuilder,
                                       LeafSystem)
from pydrake.systems.lcm import LcmPublisherSystem
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.systems.primitives import FirstOrderLowPassFilter
from pydrake.systems.sensors import ImageToLcmImageArrayT, PixelType
from pydrake.util.eigen_geometry import Isometry3

from drake.examples.manipulation_station.differential_ik import DifferentialIK

from robotlocomotion import image_array_t


# TODO(russt): Generalize this and move it to pydrake.manipulation.simple_ui.
class EndEffectorTeleop(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self._DeclareVectorOutputPort("rpy_xyz", BasicVector(6),
                                      self._DoCalcOutput)

        # Note: This timing affects the keyboard teleop performance. A larger
        #       time step causes more lag in the response.
        self._DeclarePeriodicPublish(0.01, 0.0)

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
        self.x = tk.Scale(self.window, from_=-0.6, to=0.8,
                          resolution=-1,
                          label="x",
                          length=800,
                          orient=tk.HORIZONTAL)
        self.x.pack()
        self.y = tk.Scale(self.window, from_=-0.8, to=0.3,
                          resolution=-1,
                          label="y",
                          length=800,
                          orient=tk.HORIZONTAL)
        self.y.pack()
        self.z = tk.Scale(self.window, from_=0, to=1.1,
                          resolution=-1,
                          label="z",
                          length=800,
                          orient=tk.HORIZONTAL)
        self.z.pack()

        # The key bindings below provide teleop functionality via the
        # keyboard, and are somewhat arbitrary (inspired by gaming
        # conventions). Note that in order for the keyboard bindings to
        # be active, the teleop slider window must be the active window.

        def update(scale, value):
            return lambda event: scale.set(scale.get() + value)

        # Delta displacements for motion via keyboard teleop.
        rotation_delta = 0.05  # rad
        position_delta = 0.01  # m

        # Linear motion key bindings.
        self.window.bind("<Up>", update(self.z, +position_delta))
        self.window.bind("<Down>", update(self.z, -position_delta))
        self.window.bind("<d>", update(self.y, +position_delta))
        self.window.bind("<a>", update(self.y, -position_delta))
        self.window.bind("<w>", update(self.x, +position_delta))
        self.window.bind("<s>", update(self.x, -position_delta))

        # Rotational motion key bindings.
        self.window.bind("<Control-d>", update(self.pitch, +rotation_delta))
        self.window.bind("<Control-a>", update(self.pitch, -rotation_delta))
        self.window.bind("<Control-w>", update(self.roll, +rotation_delta))
        self.window.bind("<Control-s>", update(self.roll, -rotation_delta))
        self.window.bind("<Control-Up>", update(self.yaw, +rotation_delta))
        self.window.bind("<Control-Down>", update(self.yaw, -rotation_delta))

    def SetPose(self, pose):
        """
        @param pose is an Isometry3.
        """
        tf = RigidTransform(pose)
        self.SetRPY(RollPitchYaw(tf.rotation()))
        self.SetXYZ(pose.translation())

    def SetRPY(self, rpy):
        """
        @param rpy is a RollPitchYaw object
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
    "--filter_time_const", type=float, default=0.1,
    help="Time constant for the first order low pass filter applied to"
         "the teleop commands")
parser.add_argument(
    "--velocity_limit_factor", type=float, default=1.0,
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
        ycb_objects = CreateDefaultYcbObjectList()
        for model_file, X_WObject in ycb_objects:
            station.AddManipulandFromFile(model_file, X_WObject)

    station.Finalize()

    # If using meshcat, don't render the cameras, since RgbdCamera rendering
    # only works with drake-visualizer. Without this check, running this code
    # in a docker container produces libGL errors.
    if args.meshcat:
        meshcat = builder.AddSystem(MeshcatVisualizer(
            station.get_scene_graph(), zmq_url=args.meshcat,
            open_browser=args.open_browser))
        builder.Connect(station.GetOutputPort("pose_bundle"),
                        meshcat.get_input_port(0))
    else:
        ConnectDrakeVisualizer(builder, station.get_scene_graph(),
                               station.GetOutputPort("pose_bundle"))
        image_to_lcm_image_array = builder.AddSystem(ImageToLcmImageArrayT())
        image_to_lcm_image_array.set_name("converter")
        for name in station.get_camera_names():
            cam_port = (
                image_to_lcm_image_array
                .DeclareImageInputPort[PixelType.kRgba8U]("camera_" + name))
            builder.Connect(
                station.GetOutputPort("camera_" + name + "_rgb_image"),
                cam_port)

        image_array_lcm_publisher = builder.AddSystem(
            LcmPublisherSystem.Make(
                channel="DRAKE_RGBD_CAMERA_IMAGES",
                lcm_type=image_array_t,
                lcm=None,
                publish_period=0.1,
                use_cpp_serializer=True))
        image_array_lcm_publisher.set_name("rgbd_publisher")
        builder.Connect(
            image_to_lcm_image_array.image_array_t_msg_output_port(),
            image_array_lcm_publisher.get_input_port(0))

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

teleop = builder.AddSystem(EndEffectorTeleop())
if args.test:
    teleop.window.withdraw()  # Don't display the window when testing.
filter = builder.AddSystem(
    FirstOrderLowPassFilter(time_constant=args.filter_time_const, size=6))

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

# This is important to avoid duplicate publishes to the hardware interface:
simulator.set_publish_every_time_step(False)

station_context = diagram.GetMutableSubsystemContext(
    station, simulator.get_mutable_context())

station_context.FixInputPort(station.GetInputPort(
    "iiwa_feedforward_torque").get_index(), np.zeros(7))

simulator.AdvanceTo(1e-6)
q0 = station.GetOutputPort("iiwa_position_measured").Eval(
    station_context)
differential_ik.parameters.set_nominal_joint_position(q0)

teleop.SetPose(differential_ik.ForwardKinematics(q0))
filter.set_initial_output_value(
    diagram.GetMutableSubsystemContext(
        filter, simulator.get_mutable_context()),
    teleop.get_output_port(0).Eval(diagram.GetMutableSubsystemContext(
        teleop, simulator.get_mutable_context())))
differential_ik.SetPositions(diagram.GetMutableSubsystemContext(
    differential_ik, simulator.get_mutable_context()), q0)

simulator.set_target_realtime_rate(args.target_realtime_rate)
simulator.AdvanceTo(args.duration)
