from __future__ import absolute_import, division, print_function

import unittest
import pydrake
import os.path

import pydrake.rbtree as m


class TestParsers(unittest.TestCase):
    def testAddModelInstanceFromUrdfStringSearchingInRosPackages(self):
        """Test that an instance of a URDF model can be loaded into a
        RigidBodyTree by passing a complete set of arguments to Drake's URDF
        parser.
        """
        urdf_file = os.path.join(
            pydrake.getDrakePath(),
            "examples/pr2/models/pr2_description/urdf/pr2_simplified.urdf")
        with open(urdf_file) as f:
            urdf_string = f.read()
        base_dir = os.path.dirname(urdf_file)
        # TODO(eric.cousineau): Should this be imported from `pydrake.parsers`?
        package_map = m.PackageMap()
        weld_frame = None
        floating_base_type = m.kRollPitchYaw

        robot = m.RigidBodyTree()
        m.AddModelInstanceFromUrdfStringSearchingInRosPackages(
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

    def testSdf(self):
        sdf_file = os.path.join(
            pydrake.getDrakePath(), "examples/acrobot/Acrobot.sdf")
        with open(sdf_file) as f:
            sdf_string = f.read()
        package_map = m.PackageMap()
        weld_frame = None
        floating_base_type = m.kRollPitchYaw

        robot_1 = m.RigidBodyTree()
        m.AddModelInstancesFromSdfStringSearchingInRosPackages(
            sdf_string,
            package_map,
            floating_base_type,
            weld_frame,
            robot_1)
        robot_2 = m.RigidBodyTree()
        m.AddModelInstancesFromSdfString(
            sdf_string,
            floating_base_type,
            weld_frame,
            robot_2)

        for robot in robot_1, robot_2:
            expected_num_bodies = 4
            self.assertEqual(robot.get_num_bodies(), expected_num_bodies)


if __name__ == '__main__':
    unittest.main()
