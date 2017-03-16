from __future__ import absolute_import, division, print_function
import unittest
import os.path
from pydrake.parsers import PackageMap
from pydrake import getDrakePath


class TestPackageMap(unittest.TestCase):
    def test_construction(self):
        pm = PackageMap()
        self.assertFalse(pm.Contains("foo"))
        self.assertEqual(pm.size(), 0)
        pm.Add("foo", os.path.abspath(os.curdir))
        self.assertEqual(pm.size(), 1)
        self.assertTrue(pm.Contains("foo"))
        self.assertEqual(pm.GetPath("foo"), os.path.abspath(os.curdir))

    def test_populate_from_folder(self):
        pm = PackageMap()
        self.assertEqual(pm.size(), 0)
        pm.PopulateFromFolder(os.path.join(getDrakePath(), "examples", "Atlas"))
        self.assertTrue(pm.Contains("Atlas"))
        self.assertEqual(pm.GetPath("Atlas"), os.path.join(
            getDrakePath(), "examples", "Atlas", ""))

    def test_populate_from_environment(self):
        pm = PackageMap()
        os.environ["PYDRAKE_TEST_ROS_PACKAGE_PATH"] = os.path.join(
            getDrakePath(), "examples")
        pm.PopulateFromEnvironment("PYDRAKE_TEST_ROS_PACKAGE_PATH")
        self.assertTrue(pm.Contains("Atlas"))
        self.assertEqual(pm.GetPath("Atlas"),
            os.path.join(getDrakePath(), "examples", "Atlas", ""))
        del os.environ["PYDRAKE_TEST_ROS_PACKAGE_PATH"]

    def test_populate_upstream(self):
        pm = PackageMap()
        pm.PopulateUpstreamToDrake(
            os.path.join(getDrakePath(), "examples", "Atlas", "urdf",
                         "atlas_minimal_contact.urdf"))
        self.assertTrue(pm.Contains("Atlas"))
        self.assertEqual(pm.GetPath("Atlas"),
            os.path.join(getDrakePath(), "examples", "Atlas"))


if __name__ == '__main__':
    unittest.main()
