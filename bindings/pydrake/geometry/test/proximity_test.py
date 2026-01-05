import pydrake.geometry as mut  # ruff: isort: skip

import copy
import pickle
import unittest

import numpy as np

from pydrake.common.test_utilities import numpy_compare
from pydrake.common.test_utilities.pickle_compare import assert_pickle


class TestGeometryProximity(unittest.TestCase):
    def test_plane(self):
        normal = np.array([0.0, 0.0, 1.0])
        point_on_plane = np.array([0.0, 0.0, 2.0])
        plane = mut.Plane(normal=normal, point_on_plane=point_on_plane)

        # Test CalcHeight.
        point_above = np.array([1.0, 1.0, 5.0])
        height_above = plane.CalcHeight(point=point_above)
        numpy_compare.assert_equal(height_above, 3.0)

        point_below = np.array([-1.0, -1.0, -1.0])
        height_below = plane.CalcHeight(point=point_below)
        numpy_compare.assert_equal(height_below, -3.0)

        # Test copy and pickle.
        copy.copy(plane)
        assert_pickle(
            self,
            plane,
            lambda x: f"Plane({plane.normal()!r}, {plane.point_on_plane()!r})",
        )
        # Pickle regression. The given bytestring represents a valid pickling
        # from the initial implementation. We want to ensure that the pickling
        # doesn't change, breaking old pickled values.
        pickle0 = b"\x80\x04\x95\xf9\x00\x00\x00\x00\x00\x00\x00\x8c\x10pydrake.geometry\x94\x8c\x05Plane\x94\x93\x94)\x81\x94\x8c\x15numpy.core.multiarray\x94\x8c\x0c_reconstruct\x94\x93\x94\x8c\x05numpy\x94\x8c\x07ndarray\x94\x93\x94K\x00\x85\x94C\x01b\x94\x87\x94R\x94(K\x01K\x03\x85\x94h\x07\x8c\x05dtype\x94\x93\x94\x8c\x02f8\x94\x89\x88\x87\x94R\x94(K\x03\x8c\x01<\x94NNNJ\xff\xff\xff\xffJ\xff\xff\xff\xffK\x00t\x94b\x89C\x18\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xf0?\x94t\x94bh\x06h\tK\x00\x85\x94h\x0b\x87\x94R\x94(K\x01K\x03\x85\x94h\x13\x89C\x18\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00@\x94t\x94b\x86\x94b."  # noqa
        unpickled_plane0 = pickle.loads(pickle0)
        numpy_compare.assert_equal(unpickled_plane0.normal(), plane.normal())
        numpy_compare.assert_equal(
            unpickled_plane0.point_on_plane(), plane.point_on_plane()
        )
