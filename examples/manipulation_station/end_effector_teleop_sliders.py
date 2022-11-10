import argparse
from dataclasses import dataclass
import sys
import webbrowser

import numpy as np

from pydrake.common.value import AbstractValue
from pydrake.examples import (
    ManipulationStation, ManipulationStationHardwareInterface,
    CreateClutterClearingYcbObjectList, SchunkCollisionModel)
from pydrake.geometry import DrakeVisualizer, Meshcat, MeshcatVisualizer
from pydrake.manipulation.planner import (
    DifferentialInverseKinematicsIntegrator,
    DifferentialInverseKinematicsParameters)
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import (DiagramBuilder, LeafSystem,
                                       PublishEvent)
from pydrake.systems.lcm import LcmPublisherSystem
from pydrake.systems.primitives import FirstOrderLowPassFilter, VectorLogSink
from pydrake.systems.sensors import ImageToLcmImageArrayT, PixelType

from drake.examples.manipulation_station.schunk_wsg_buttons import \
    SchunkWsgButtons

from drake import lcmt_image_array


class EndEffectorTeleop(LeafSystem):
    @dataclass
    class SliderDefault:
        """Default values for the meshcat sliders."""
        name: str
        """The name that is used to add / query values from."""
        default: float
        """The initial value of the slider."""

    _ROLL = SliderDefault("Roll", 0.0)
    _PITCH = SliderDefault("Pitch", 0.0)
    _YAW = SliderDefault("Yaw", 1.57)
    _X = SliderDefault("X", 0.0)
    _Y = SliderDefault("Y", 0.0)
    _Z = SliderDefault("Z", 0.0)

    def __init__(self, meshcat, planar=False):
        """
        @param meshcat The already created pydrake.geometry.Meshcat instance.
        @param planar if True, the GUI will not have Pitch, Yaw, or Y-axis
                      sliders and default values will be returned.
        """

        LeafSystem.__init__(self)
        # Note: Disable caching because meshcat's sliders have undeclared
        # state.
        self.DeclareVectorOutputPort(
            "rpy_xyz", 6, self.DoCalcOutput).disable_caching_by_default()
        self.meshcat = meshcat
        self.planar = planar

        # Rotation control sliders.
        self.meshcat.AddSlider(
            name=self._ROLL.name, min=-2.0 * np.pi, max=2.0 * np.pi, step=0.01,
            value=self._ROLL.default)
        if not self.planar:
            self.meshcat.AddSlider(
                name=self._PITCH.name, min=-2.0 * np.pi, max=2.0 * np.pi,
                step=0.01, value=self._PITCH.default)
            self.meshcat.AddSlider(
                name=self._YAW.name, min=-2.0 * np.pi, max=2.0 * np.pi,
                step=0.01, value=self._YAW.default)

        # Position control sliders.
        self.meshcat.AddSlider(
            name=self._X.name, min=-0.6, max=0.8, step=0.01,
            value=self._X.default)
        if not self.planar:
            self.meshcat.AddSlider(
                name=self._Y.name, min=-0.8, max=0.3, step=0.01,
                value=self._Y.default)
        self.meshcat.AddSlider(
            name=self._Z.name, min=0.0, max=1.1, step=0.01,
            value=self._Z.default)

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
        self.meshcat.SetSliderValue(self._ROLL.name, rpy.roll_angle())
        if not self.planar:
            self.meshcat.SetSliderValue(self._PITCH.name, rpy.pitch_angle())
            self.meshcat.SetSliderValue(self._YAW.name, rpy.yaw_angle())

    def SetXYZ(self, xyz):
        """
        @param xyz is a 3 element vector of x, y, z.
        """
        self.meshcat.SetSliderValue(self._X.name, xyz[0])
        if not self.planar:
            self.meshcat.SetSliderValue(self._Y.name, xyz[1])
        self.meshcat.SetSliderValue(self._Z.name, xyz[2])

    def DoCalcOutput(self, context, output):
        roll = self.meshcat.GetSliderValue(self._ROLL.name)
        if not self.planar:
            pitch = self.meshcat.GetSliderValue(self._PITCH.name)
            yaw = self.meshcat.GetSliderValue(self._YAW.name)
        else:
            pitch = self._PITCH.default
            yaw = self._YAW.default
        x = self.meshcat.GetSliderValue(self._X.name)
        if not self.planar:
            y = self.meshcat.GetSliderValue(self._Y.name)
        else:
            y = self._Y.default
        z = self.meshcat.GetSliderValue(self._Z.name)

        output.SetAtIndex(0, roll)
        output.SetAtIndex(1, pitch)
        output.SetAtIndex(2, yaw)
        output.SetAtIndex(3, x)
        output.SetAtIndex(4, y)
        output.SetAtIndex(5, z)


class ToPose(LeafSystem):
    def __init__(self, grab_focus=True):
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("rpy_xyz", 6)
        self.DeclareAbstractOutputPort(
            "pose", lambda: AbstractValue.Make(RigidTransform()),
            self.DoCalcOutput)

    def DoCalcOutput(self, context, output):
        rpy_xyz = self.get_input_port().Eval(context)
        output.set_value(RigidTransform(RollPitchYaw(rpy_xyz[:3]),
                                        rpy_xyz[3:]))


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
    parser.add_argument(
        "-w", "--open-window", dest="browser_new",
        action="store_const", const=1, default=None,
        help=(
            "Open the MeshCat display in a new browser window.  NOTE: the "
            "slider controls are available in the meshcat viewer by clicking "
            "'Open Controls' in the top-right corner."))
    args = parser.parse_args()

    builder = DiagramBuilder()

    # NOTE: the meshcat instance is always created in order to create the
    # teleop controls (orientation sliders and open/close gripper button). When
    # args.hardware is True, the meshcat server will *not* display robot
    # geometry, but it will contain the joint sliders and open/close gripper
    # button in the "Open Controls" tab in the top-right of the viewing server.
    meshcat = Meshcat()

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

        # Connect the meshcat visualizer.
        meshcat_visualizer = MeshcatVisualizer.AddToBuilder(
            builder=builder,
            query_object_port=geometry_query_port,
            meshcat=meshcat)

        # Configure the planar visualization.
        if args.setup == 'planar':
            meshcat.Set2dRenderMode()

        # Connect and publish to drake visualizer.
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

    if args.browser_new is not None:
        url = meshcat.web_url()
        webbrowser.open(url=url, new=args.browser_new)

    robot = station.get_controller_plant()
    params = DifferentialInverseKinematicsParameters(robot.num_positions(),
                                                     robot.num_velocities())

    time_step = 0.005
    params.set_time_step(time_step)
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
    params.set_joint_velocity_limits(
        (-factor * iiwa14_velocity_limits, factor * iiwa14_velocity_limits))
    differential_ik = builder.AddSystem(
        DifferentialInverseKinematicsIntegrator(
            robot, robot.GetFrameByName("iiwa_link_7"), time_step, params))

    builder.Connect(differential_ik.GetOutputPort("joint_positions"),
                    station.GetInputPort("iiwa_position"))

    teleop = builder.AddSystem(EndEffectorTeleop(
        meshcat, args.setup == 'planar'))
    filter = builder.AddSystem(
        FirstOrderLowPassFilter(time_constant=args.filter_time_const, size=6))

    builder.Connect(teleop.get_output_port(0), filter.get_input_port(0))

    to_pose = builder.AddSystem(ToPose())
    builder.Connect(filter.get_output_port(0),
                    to_pose.get_input_port())
    builder.Connect(to_pose.get_output_port(),
                    differential_ik.GetInputPort("X_WE_desired"))

    wsg_buttons = builder.AddSystem(SchunkWsgButtons(meshcat=meshcat))
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
    differential_ik.get_mutable_parameters().set_nominal_joint_position(q0)

    differential_ik.SetPositions(
        differential_ik.GetMyMutableContextFromRoot(
            simulator.get_mutable_context()), q0)
    teleop.SetPose(
        differential_ik.ForwardKinematics(
            differential_ik.GetMyContextFromRoot(simulator.get_context())))
    filter.set_initial_output_value(
        diagram.GetMutableSubsystemContext(
            filter, simulator.get_mutable_context()),
        teleop.get_output_port(0).Eval(diagram.GetMutableSubsystemContext(
            teleop, simulator.get_mutable_context())))

    simulator.set_target_realtime_rate(args.target_realtime_rate)
    simulator.AdvanceTo(args.duration)

    # Ensure that our initialization logic was correct, by inspecting our
    # logged joint velocities.
    if args.test:
        iiwa_velocities_log = iiwa_velocities.FindLog(simulator.get_context())
        for time, qdot in zip(iiwa_velocities_log.sample_times(),
                              iiwa_velocities_log.data().transpose()):
            # TODO(jwnimmer-tri) We should be able to do better than a 40
            # rad/sec limit, but that's the best we can enforce for now.
            if qdot.max() > 0.1:
                print(f"ERROR: large qdot {qdot} at time {time}")
                sys.exit(1)


if __name__ == '__main__':
    main()
