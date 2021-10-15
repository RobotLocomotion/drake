import argparse
import sys

if sys.platform == "darwin":
    # TODO(jamiesnape): Fix this example on macOS Big Sur. Skipping on all
    # macOS for simplicity and because of the tendency for macOS versioning
    # schemes to unexpectedly change.
    # ImportError: C++ type is not registered in pybind:
    # NSt3__112basic_stringIcNS_11char_traitsIcEENS_9allocatorIcEEEE
    print("ERROR: Skipping this example on macOS because it fails on Big Sur")
    sys.exit(0)

try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk
import numpy as np

from pydrake.examples.manipulation_station import (
    ManipulationStation, ManipulationStationHardwareInterface,
    CreateClutterClearingYcbObjectList, SchunkCollisionModel)
from pydrake.geometry import DrakeVisualizer
from pydrake.manipulation.simple_ui import SchunkWsgButtons
from pydrake.manipulation.planner import (
    DifferentialInverseKinematicsParameters)
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import (DiagramBuilder, LeafSystem,
                                       PublishEvent)
from pydrake.systems.lcm import LcmPublisherSystem
from pydrake.systems.meshcat_visualizer import (
    ConnectMeshcatVisualizer, MeshcatVisualizer)
from pydrake.systems.primitives import FirstOrderLowPassFilter, VectorLogSink
from pydrake.systems.sensors import ImageToLcmImageArrayT, PixelType
from pydrake.systems.planar_scenegraph_visualizer import \
    ConnectPlanarSceneGraphVisualizer

from drake.examples.manipulation_station.differential_ik import DifferentialIK

from drake import lcmt_image_array


# TODO(russt): Generalize this and move it to pydrake.manipulation.simple_ui.
class EndEffectorTeleop(LeafSystem):
    def __init__(self, planar=False):
        """
        @param planar if True, restricts the GUI and the output to have y=0,
                      roll=0, yaw=0.
        """

        LeafSystem.__init__(self)
        self.DeclareVectorOutputPort("rpy_xyz", 6,
                                     self.DoCalcOutput)

        # Note: This timing affects the keyboard teleop performance. A larger
        #       time step causes more lag in the response.
        self.DeclarePeriodicEvent(0.01, 0.0, PublishEvent(self._update_window))
        self.planar = planar

        self.window = tk.Tk()
        self.window.title("End-Effector TeleOp")

        self.roll = tk.Scale(self.window, from_=-2 * np.pi, to=2 * np.pi,
                             resolution=-1,
                             label="roll (keys: ctrl-right, ctrl-left)",
                             length=800,
                             orient=tk.HORIZONTAL)
        self.roll.pack()
        self.roll.set(0)
        self.pitch = tk.Scale(self.window, from_=-2 * np.pi, to=2 * np.pi,
                              resolution=-1,
                              label="pitch (keys: ctrl-d, ctrl-a)",
                              length=800,
                              orient=tk.HORIZONTAL)
        if not planar:
            self.pitch.pack()
        self.pitch.set(0)
        self.yaw = tk.Scale(self.window, from_=-2 * np.pi, to=2 * np.pi,
                            resolution=-1,
                            label="yaw (keys: ctrl-up, ctrl-down)",
                            length=800,
                            orient=tk.HORIZONTAL)
        if not planar:
            self.yaw.pack()
        self.yaw.set(1.57)
        self.x = tk.Scale(self.window, from_=-0.6, to=0.8,
                          resolution=-1,
                          label="x (keys: right, left)",
                          length=800,
                          orient=tk.HORIZONTAL)
        self.x.pack()
        self.x.set(0)
        self.y = tk.Scale(self.window, from_=-0.8, to=0.3,
                          resolution=-1,
                          label="y (keys: d, a)",
                          length=800,
                          orient=tk.HORIZONTAL)
        if not planar:
            self.y.pack()
        self.y.set(0)
        self.z = tk.Scale(self.window, from_=0, to=1.1,
                          resolution=-1,
                          label="z (keys: up, down)",
                          length=800,
                          orient=tk.HORIZONTAL)
        self.z.pack()
        self.z.set(0)

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
        if (not planar):
            self.window.bind("<d>", update(self.y, +position_delta))
            self.window.bind("<a>", update(self.y, -position_delta))
        self.window.bind("<Right>", update(self.x, +position_delta))
        self.window.bind("<Left>", update(self.x, -position_delta))

        # Rotational motion key bindings.
        self.window.bind("<Control-Right>", update(self.roll, +rotation_delta))
        self.window.bind("<Control-Left>", update(self.roll, -rotation_delta))
        if (not planar):
            self.window.bind("<Control-d>",
                             update(self.pitch, +rotation_delta))
            self.window.bind("<Control-a>",
                             update(self.pitch, -rotation_delta))
            self.window.bind("<Control-Up>",
                             update(self.yaw, +rotation_delta))
            self.window.bind("<Control-Down>",
                             update(self.yaw, -rotation_delta))

    def SetPose(self, pose):
        """
        @param pose is a RigidTransform or else any type accepted by
                    RigidTransform's constructor
        """
        tf = RigidTransform(pose)
        self.SetRPY(RollPitchYaw(tf.rotation()))
        self.SetXYZ(tf.translation())

    def SetRPY(self, rpy):
        """
        @param rpy is a RollPitchYaw object
        """
        self.roll.set(rpy.roll_angle())
        if not self.planar:
            self.pitch.set(rpy.pitch_angle())
            self.yaw.set(rpy.yaw_angle())

    def SetXYZ(self, xyz):
        """
        @param xyz is a 3 element vector of x, y, z.
        """
        self.x.set(xyz[0])
        if not self.planar:
            self.y.set(xyz[1])
        self.z.set(xyz[2])

    def _update_window(self, context, event):
        self.window.update_idletasks()
        self.window.update()

    def DoCalcOutput(self, context, output):
        output.SetAtIndex(0, self.roll.get())
        output.SetAtIndex(1, self.pitch.get())
        output.SetAtIndex(2, self.yaw.get())
        output.SetAtIndex(3, self.x.get())
        output.SetAtIndex(4, self.y.get())
        output.SetAtIndex(5, self.z.get())


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
        "--filter_time_const", type=float, default=0.1,
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
        choices=['manipulation_class', 'clutter_clearing', 'planar'])
    parser.add_argument(
        '--schunk_collision_model', type=str, default='box',
        help="The Schunk collision model to use for simulation. ",
        choices=['box', 'box_plus_fingertip_spheres'])
    MeshcatVisualizer.add_argparse_argument(parser)
    args = parser.parse_args()

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
                "drake/examples/manipulation_station/models/"
                + "061_foam_brick.sdf",
                RigidTransform(RotationMatrix.Identity(), [0.6, 0, 0]))
        elif args.setup == 'clutter_clearing':
            station.SetupClutterClearingStation(
                schunk_model=schunk_model)
            ycb_objects = CreateClutterClearingYcbObjectList()
            for model_file, X_WObject in ycb_objects:
                station.AddManipulandFromFile(model_file, X_WObject)
        elif args.setup == 'planar':
            station.SetupPlanarIiwaStation(
                schunk_model=schunk_model)
            station.AddManipulandFromFile(
                "drake/examples/manipulation_station/models/"
                + "061_foam_brick.sdf",
                RigidTransform(RotationMatrix.Identity(), [0.6, 0, 0]))

        station.Finalize()

        # If using meshcat, don't render the cameras, since RgbdCamera
        # rendering only works with drake-visualizer. Without this check,
        # running this code in a docker container produces libGL errors.
        geometry_query_port = station.GetOutputPort("geometry_query")
        if args.meshcat:
            meshcat = ConnectMeshcatVisualizer(
                builder, output_port=geometry_query_port,
                zmq_url=args.meshcat, open_browser=args.open_browser)
            if args.setup == 'planar':
                meshcat.set_planar_viewpoint()

        elif args.setup == 'planar':
            ConnectPlanarSceneGraphVisualizer(
                builder, station.get_scene_graph(), geometry_query_port)

        else:
            DrakeVisualizer.AddToBuilder(builder, geometry_query_port)
            image_to_lcm_image_array = builder.AddSystem(
                ImageToLcmImageArrayT())
            image_to_lcm_image_array.set_name("converter")
            for name in station.get_camera_names():
                cam_port = (
                    image_to_lcm_image_array
                    .DeclareImageInputPort[PixelType.kRgba8U](
                        "camera_" + name))
                builder.Connect(
                    station.GetOutputPort("camera_" + name + "_rgb_image"),
                    cam_port)

            image_array_lcm_publisher = builder.AddSystem(
                LcmPublisherSystem.Make(
                    channel="DRAKE_RGBD_CAMERA_IMAGES",
                    lcm_type=lcmt_image_array,
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
    if args.setup == 'planar':
        # Extract the 3 joints that are not welded in the planar version.
        iiwa14_velocity_limits = iiwa14_velocity_limits[1:6:2]
        # The below constant is in body frame.
        params.set_end_effector_velocity_gain([1, 0, 0, 0, 1, 1])
    # Stay within a small fraction of those limits for this teleop demo.
    factor = args.velocity_limit_factor
    params.set_joint_velocity_limits((-factor*iiwa14_velocity_limits,
                                      factor*iiwa14_velocity_limits))
    differential_ik = builder.AddSystem(DifferentialIK(
        robot, robot.GetFrameByName("iiwa_link_7"), params, time_step))

    builder.Connect(differential_ik.GetOutputPort("joint_position_desired"),
                    station.GetInputPort("iiwa_position"))

    teleop = builder.AddSystem(EndEffectorTeleop(args.setup == 'planar'))
    if args.test:
        teleop.window.withdraw()  # Don't display the window when testing.
    filter = builder.AddSystem(
        FirstOrderLowPassFilter(time_constant=args.filter_time_const, size=6))

    builder.Connect(teleop.get_output_port(0), filter.get_input_port(0))
    builder.Connect(filter.get_output_port(0),
                    differential_ik.GetInputPort("rpy_xyz_desired"))

    wsg_buttons = builder.AddSystem(SchunkWsgButtons(teleop.window))
    builder.Connect(wsg_buttons.GetOutputPort("position"),
                    station.GetInputPort("wsg_position"))
    builder.Connect(wsg_buttons.GetOutputPort("force_limit"),
                    station.GetInputPort("wsg_force_limit"))

    # When in regression test mode, log our joint velocities to later check
    # that they were sufficiently quiet.
    num_iiwa_joints = station.num_iiwa_joints()
    if args.test:
        iiwa_velocities = builder.AddSystem(VectorLogSink(num_iiwa_joints))
        builder.Connect(station.GetOutputPort("iiwa_velocity_estimated"),
                        iiwa_velocities.get_input_port(0))
    else:
        iiwa_velocities = None

    diagram = builder.Build()
    simulator = Simulator(diagram)
    iiwa_velocities_log = iiwa_velocities.FindLog(simulator.get_context())

    # This is important to avoid duplicate publishes to the hardware interface:
    simulator.set_publish_every_time_step(False)

    station_context = diagram.GetMutableSubsystemContext(
        station, simulator.get_mutable_context())

    station.GetInputPort("iiwa_feedforward_torque").FixValue(
        station_context, np.zeros(num_iiwa_joints))

    # If the diagram is only the hardware interface, then we must advance it a
    # little bit so that first LCM messages get processed. A simulated plant is
    # already publishing correct positions even without advancing, and indeed
    # we must not advance a simulated plant until the sliders and filters have
    # been initialized to match the plant.
    if args.hardware:
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

    # Ensure that our initialization logic was correct, by inspecting our
    # logged joint velocities.
    if args.test:
        for time, qdot in zip(iiwa_velocities_log.sample_times(),
                              iiwa_velocities_log.data().transpose()):
            # TODO(jwnimmer-tri) We should be able to do better than a 40
            # rad/sec limit, but that's the best we can enforce for now.
            if qdot.max() > 0.1:
                print(f"ERROR: large qdot {qdot} at time {time}")
                sys.exit(1)


if __name__ == '__main__':
    main()
