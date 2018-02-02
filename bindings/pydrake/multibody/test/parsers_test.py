from __future__ import absolute_import, division, print_function

import unittest
import os.path

from pydrake import getDrakePath
from pydrake.multibody.parsers import PackageMap
from pydrake.multibody.rigid_body_tree import (
    AddModelInstanceFromUrdfStringSearchingInRosPackages,
    AddModelInstancesFromSdfString,
    AddModelInstancesFromSdfStringSearchingInRosPackages,
    FloatingBaseType,
    RigidBodyTree,
)


class TestParsers(unittest.TestCase):
    def test_urdf(self):
        """Test that an instance of a URDF model can be loaded into a
        RigidBodyTree by passing a complete set of arguments to Drake's URDF
        parser.
        """
        urdf_file = os.path.join(
            getDrakePath(),
            "examples/pr2/models/pr2_description/urdf/pr2_simplified.urdf")
        with open(urdf_file) as f:
            urdf_string = f.read()
        base_dir = os.path.dirname(urdf_file)
        package_map = PackageMap()
        weld_frame = None
        floating_base_type = FloatingBaseType.kRollPitchYaw

        robot = RigidBodyTree()
        AddModelInstanceFromUrdfStringSearchingInRosPackages(
            urdf_string,
            package_map,
            base_dir,
            floating_base_type,
            weld_frame,
            robot)

        expected_num_bodies = 86
        self.assertEqual(robot.get_num_bodies(), expected_num_bodies,
                         msg='Incorrect number of bodies: {0} vs. {1}'.format(
                             robot.get_num_bodies(), expected_num_bodies))

    def test_sdf(self):
        sdf_file = os.path.join(
            getDrakePath(), "examples/acrobot/Acrobot.sdf")
        with open(sdf_file) as f:
            sdf_string = f.read()
        package_map = PackageMap()
        weld_frame = None
        floating_base_type = FloatingBaseType.kRollPitchYaw

        robot_1 = RigidBodyTree()
        AddModelInstancesFromSdfStringSearchingInRosPackages(
            sdf_string,
            package_map,
            floating_base_type,
            weld_frame,
            robot_1)
        robot_2 = RigidBodyTree()
        AddModelInstancesFromSdfString(
            sdf_string,
            floating_base_type,
            weld_frame,
            robot_2)

        for robot in robot_1, robot_2:
            expected_num_bodies = 4
            self.assertEqual(robot.get_num_bodies(), expected_num_bodies)

    def test_package_map(self):
        pm = PackageMap()
        self.assertFalse(pm.Contains("foo"))
        self.assertEqual(pm.size(), 0)
        pm.Add("foo", os.path.abspath(os.curdir))
        self.assertEqual(pm.size(), 1)
        self.assertTrue(pm.Contains("foo"))
        self.assertEqual(pm.GetPath("foo"), os.path.abspath(os.curdir))

        # Populate from folder.
        # TODO(eric.cousineau): This mismatch between casing is confusing, with
        # `Atlas` being the package name, but `atlas` being the dirctory name.
        pm = PackageMap()
        self.assertEqual(pm.size(), 0)
        pm.PopulateFromFolder(
            os.path.join(getDrakePath(), "examples", "atlas"))
        self.assertTrue(pm.Contains("Atlas"))
        self.assertEqual(pm.GetPath("Atlas"), os.path.join(
            getDrakePath(), "examples", "atlas", ""))

        # Populate from environment.
        pm = PackageMap()
        os.environ["PYDRAKE_TEST_ROS_PACKAGE_PATH"] = os.path.join(
            getDrakePath(), "examples")
        pm.PopulateFromEnvironment("PYDRAKE_TEST_ROS_PACKAGE_PATH")
        self.assertTrue(pm.Contains("Atlas"))
        self.assertEqual(pm.GetPath("Atlas"), os.path.join(
            getDrakePath(), "examples", "atlas", ""))
        del os.environ["PYDRAKE_TEST_ROS_PACKAGE_PATH"]

        # Populate upstream.
        pm = PackageMap()
        pm.PopulateUpstreamToDrake(
            os.path.join(getDrakePath(), "examples", "atlas", "urdf",
                         "atlas_minimal_contact.urdf"))
        self.assertTrue(pm.Contains("Atlas"))
        self.assertEqual(pm.GetPath("Atlas"), os.path.join(
            getDrakePath(), "examples", "atlas"))


if __name__ == '__main__':
    unittest.main()
