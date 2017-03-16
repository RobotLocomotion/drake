from __future__ import absolute_import, division, print_function
import unittest
import os.path

from pydrake.parsers import PackageMap
from pydrake import rbtree
from pydrake import getDrakePath


class TestAtlasConstructor(unittest.TestCase):
    def test_constructor(self):
        pm = PackageMap()
        model = os.path.join(getDrakePath(),
            "examples", "Atlas", "urdf", "atlas_minimal_contact.urdf")
        pm.PopulateUpstreamToDrake(model)
        robot = rbtree.RigidBodyTree(
            model, package_map=pm,
            floating_base_type=rbtree.FloatingBaseType.kRollPitchYaw)


if __name__ == '__main__':
    unittest.main()
