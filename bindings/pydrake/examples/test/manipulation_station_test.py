
import unittest
import numpy as np

from pydrake.examples.manipulation_station import (
    IiwaCollisionModel,
    ManipulationStation,
    ManipulationStationHardwareInterface
)
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant


class TestManipulationStation(unittest.TestCase):
    def test_manipulation_station(self):
        # Just check the spelling.
        station = ManipulationStation(
            time_step=0.001, collision_model=IiwaCollisionModel.kNoCollision)
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
        station.SetIiwaPosition(q, context)
        np.testing.assert_array_equal(q, station.GetIiwaPosition(context))
        station.SetIiwaVelocity(v, context)
        np.testing.assert_array_equal(v, station.GetIiwaVelocity(context))

        q = 4.23
        v = 8.51
        station.SetWsgPosition(q, context)
        self.assertEqual(q, station.GetWsgPosition(context))
        station.SetWsgVelocity(v, context)
        self.assertEqual(v, station.GetWsgVelocity(context))

        station.get_camera_poses_in_world()["0"]
        self.assertEqual(len(station.get_camera_names()), 3)

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
