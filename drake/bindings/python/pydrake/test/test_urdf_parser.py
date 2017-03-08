from __future__ import absolute_import, division, print_function

import unittest
import pydrake
import os.path


class TestUrdfParser(unittest.TestCase):
    """Test that an instance of a URDF model can be loaded into a
    RigidBodyTree by passing a complete set of arguments to Drake's URDF
    parser.
    """
    def testAddModelInstanceFromUrdfStringSearchingInRosPackages(self):
        urdf_file = os.path.join(pydrake.getDrakePath(),
                                 "examples/PR2/pr2.urdf")
        urdf_string = open(urdf_file).read()
        base_dir = os.path.dirname(urdf_file)
        package_map = pydrake.rbtree.PackageMap()
        weld_frame = None
        floating_base_type = pydrake.rbtree.kRollPitchYaw

        robot = pydrake.rbtree.RigidBodyTree()
        pydrake.rbtree.AddModelInstanceFromUrdfStringSearchingInRosPackages(
            urdf_string,
            package_map,
            base_dir,
            floating_base_type,
            weld_frame,
            robot)

        expected_num_bodies = 83
        self.assertEqual(robot.get_num_bodies(), expected_num_bodies,
                         msg='Incorrect number of bodies: {0} vs. {1}'.format(
                             robot.get_num_bodies(), expected_num_bodies))

if __name__ == '__main__':
    unittest.main()
