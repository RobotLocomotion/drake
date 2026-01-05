import pydrake.geometry as mut  # ruff: isort: skip

import copy
import unittest

import numpy as np

from pydrake.common.test_utilities import numpy_compare
from pydrake.common.test_utilities.pickle_compare import assert_pickle


class TestGeometryProximity(unittest.TestCase):
    def test_plane(self):
        normal = np.array([0.0, 0.0, 1.0])
        point_on_plane = np.array([0.0, 0.0, 2.0])
        plane = mut.Plane(normal, point_on_plane)

        # Test CalcHeight.
        point_above = np.array([1.0, 1.0, 5.0])
        height_above = plane.CalcHeight(point_above)
        numpy_compare.assert_equal(height_above, 3.0)

        point_below = np.array([-1.0, -1.0, -1.0])
        height_below = plane.CalcHeight(point_below)
        numpy_compare.assert_equal(height_below, -3.0)

        # Test copy and pickle.
        copy.copy(plane)
        assert_pickle(
            self,
            plane,
            lambda x: f"Plane({plane.normal()}, {plane.point_on_plane()})",
        )
