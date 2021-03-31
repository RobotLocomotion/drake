import unittest
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.examples.manipulation_station import (
    CreateClutterClearingYcbObjectList,
    CreateManipulationClassYcbObjectList,
    IiwaCollisionModel,
    ManipulationStation,
    ManipulationStationHardwareInterface
)
from pydrake.geometry.render import (
    ClippingRange,
    ColorRenderCamera,
    DepthRange,
    DepthRenderCamera,
    RenderCameraCore,
)
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.multibody.parsing import Parser
from pydrake.systems.sensors import CameraInfo


class TestManipulationStation(unittest.TestCase):
    def test_manipulation_station(self):
        # Just check the spelling.
        station = ManipulationStation(time_step=0.001)
        station.SetupManipulationClassStation()
        station.SetWsgGains(0.1, 0.1)
        station.SetIiwaPositionGains(np.ones(7))
        station.SetIiwaVelocityGains(np.ones(7))
        station.SetIiwaIntegralGains(np.ones(7))
        station.Finalize()
        station.get_multibody_plant()
        station.get_mutable_multibody_plant()
        station.get_scene_graph()
        station.get_mutable_scene_graph()
        station.get_controller_plant()

        # Check the setters/getters.
        context = station.CreateDefaultContext()
        q = np.linspace(0.04, 0.6, num=7)
        v = np.linspace(-2.3, 0.5, num=7)
        station.SetIiwaPosition(context, q)
        np.testing.assert_array_equal(q, station.GetIiwaPosition(context))
        station.SetIiwaVelocity(context, v)
        np.testing.assert_array_equal(v, station.GetIiwaVelocity(context))

        q = 0.0423
        v = 0.0851
        station.SetWsgPosition(context, q)
        self.assertEqual(q, station.GetWsgPosition(context))
        station.SetWsgVelocity(context, v)
        self.assertEqual(v, station.GetWsgVelocity(context))

        station.GetStaticCameraPosesInWorld()["0"]
        self.assertEqual(len(station.get_camera_names()), 3)

    def test_manipulation_station_add_iiwa_and_wsg_explicitly(self):
        station = ManipulationStation()
        parser = Parser(station.get_mutable_multibody_plant(),
                        station.get_mutable_scene_graph())
        plant = station.get_mutable_multibody_plant()

        # Add models for iiwa and wsg
        iiwa_model_file = FindResourceOrThrow(
            "drake/manipulation/models/iiwa_description/iiwa7/"
            "iiwa7_no_collision.sdf")
        iiwa = parser.AddModelFromFile(iiwa_model_file, "iiwa")
        X_WI = RigidTransform.Identity()
        plant.WeldFrames(plant.world_frame(),
                         plant.GetFrameByName("iiwa_link_0", iiwa),
                         X_WI)

        wsg_model_file = FindResourceOrThrow(
            "drake/manipulation/models/wsg_50_description/sdf/"
            "schunk_wsg_50.sdf")
        wsg = parser.AddModelFromFile(wsg_model_file, "gripper")
        X_7G = RigidTransform.Identity()
        plant.WeldFrames(
            plant.GetFrameByName("iiwa_link_7", iiwa),
            plant.GetFrameByName("body", wsg),
            X_7G)

        # Register models for the controller.
        station.RegisterIiwaControllerModel(
            iiwa_model_file, iiwa, plant.world_frame(),
            plant.GetFrameByName("iiwa_link_0", iiwa), X_WI)
        station.RegisterWsgControllerModel(
            wsg_model_file, wsg,
            plant.GetFrameByName("iiwa_link_7", iiwa),
            plant.GetFrameByName("body", wsg), X_7G)

        # Finalize
        station.Finalize()
        self.assertEqual(station.num_iiwa_joints(), 7)

        # This WSG gripper model has 2 independent dof, and the IIWA model
        # has 7.
        self.assertEqual(plant.num_positions(), 9)
        self.assertEqual(plant.num_velocities(), 9)

    def test_clutter_clearing_setup(self):
        station = ManipulationStation(time_step=0.001)
        station.SetupClutterClearingStation()

        num_station_bodies = (
            station.get_multibody_plant().num_model_instances())

        ycb_objects = CreateClutterClearingYcbObjectList()
        for model_file, X_WObject in ycb_objects:
            station.AddManipulandFromFile(model_file, X_WObject)

        station.Finalize()
        self.assertEqual(station.num_iiwa_joints(), 7)

        context = station.CreateDefaultContext()
        q = np.linspace(0.04, 0.6, num=7)
        v = np.linspace(-2.3, 0.5, num=7)
        station.SetIiwaPosition(context, q)
        np.testing.assert_array_equal(q, station.GetIiwaPosition(context))
        station.SetIiwaVelocity(context, v)
        np.testing.assert_array_equal(v, station.GetIiwaVelocity(context))

        q = 0.0423
        v = 0.0851
        station.SetWsgPosition(context, q)
        self.assertEqual(q, station.GetWsgPosition(context))
        station.SetWsgVelocity(context, v)
        self.assertEqual(v, station.GetWsgVelocity(context))

        self.assertEqual(len(station.get_camera_names()), 1)
        self.assertEqual(station.get_multibody_plant().num_model_instances(),
                         num_station_bodies + len(ycb_objects))

    def test_planar_iiwa_setup(self):
        station = ManipulationStation(time_step=0.001)
        station.SetupPlanarIiwaStation()
        station.Finalize()
        self.assertEqual(station.num_iiwa_joints(), 3)

        context = station.CreateDefaultContext()
        q = np.linspace(0.04, 0.6, num=3)
        v = np.linspace(-2.3, 0.5, num=3)
        station.SetIiwaPosition(context, q)
        np.testing.assert_array_equal(q, station.GetIiwaPosition(context))
        station.SetIiwaVelocity(context, v)
        np.testing.assert_array_equal(v, station.GetIiwaVelocity(context))

        q = 0.0423
        v = 0.0851
        station.SetWsgPosition(context, q)
        self.assertEqual(q, station.GetWsgPosition(context))
        station.SetWsgVelocity(context, v)
        self.assertEqual(v, station.GetWsgVelocity(context))

    def test_iiwa_collision_model(self):
        # Check that all of the elements of the enum were spelled correctly.
        IiwaCollisionModel.kNoCollision
        IiwaCollisionModel.kBoxCollision

    def test_manipulation_station_hardware_interface(self):
        station = ManipulationStationHardwareInterface(
            camera_names=["123", "456"])
        # Don't actually call Connect here, since it would block.
        station.get_controller_plant()
        self.assertEqual(len(station.get_camera_names()), 2)
        self.assertEqual(station.num_iiwa_joints(), 7)

    def test_ycb_object_creation(self):
        ycb_objects = CreateClutterClearingYcbObjectList()
        self.assertEqual(len(ycb_objects), 6)

        ycb_objects = CreateManipulationClassYcbObjectList()
        self.assertEqual(len(ycb_objects), 5)

    def test_rgbd_sensor_registration(self):
        X_PC = RigidTransform(p=[1, 2, 3])
        station = ManipulationStation(time_step=0.001)
        station.SetupManipulationClassStation()
        plant = station.get_multibody_plant()
        color_camera = ColorRenderCamera(
            RenderCameraCore("renderer", CameraInfo(10, 10, np.pi/4),
                             ClippingRange(0.1, 10.0), RigidTransform()),
            False)
        depth_camera = DepthRenderCamera(color_camera.core(),
                                         DepthRange(0.1, 9.5))
        station.RegisterRgbdSensor("single_sensor", plant.world_frame(), X_PC,
                                   depth_camera)
        station.RegisterRgbdSensor("dual_sensor", plant.world_frame(), X_PC,
                                   color_camera, depth_camera)
        station.Finalize()
        camera_poses = station.GetStaticCameraPosesInWorld()
        # The three default cameras plus the two just added.
        self.assertEqual(len(camera_poses), 5)
        self.assertTrue("single_sensor" in camera_poses)
        self.assertTrue("dual_sensor" in camera_poses)
