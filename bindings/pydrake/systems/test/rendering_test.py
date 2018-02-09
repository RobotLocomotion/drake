# -*- coding: utf-8 -*-

from pydrake.systems.rendering import (
    PoseVector,
)

import copy
import unittest
import numpy as np

from pydrake.systems.framework import (
    BasicVector,
)
from pydrake.util.eigen_geometry import (
    Isometry3,
    Quaternion,
)


def normalized(x):
    return x / np.linalg.norm(x)


class TestRendering(unittest.TestCase):
    def test_pose_vector(self):
        value = PoseVector()
        self.assertTrue(isinstance(value, BasicVector))
        self.assertTrue(isinstance(copy.copy(value), PoseVector))
        self.assertTrue(isinstance(value.Clone(), PoseVector))
        self.assertEquals(value.size(), PoseVector.kSize)
        # - Accessors.
        self.assertTrue(isinstance(
            value.get_isometry(), Isometry3))
        self.assertTrue(isinstance(
            value.get_rotation(), Quaternion))
        self.assertTrue(isinstance(
            value.get_translation(), np.ndarray))
        # - - Value.
        self.assertTrue(np.allclose(
            value.get_isometry().matrix(), np.eye(4, 4)))
        # - Mutators.
        p = [0, 1, 2]
        q = Quaternion(wxyz=normalized([0.1, 0.3, 0.7, 0.9]))
        X_expected = Isometry3(q, p)
        value.set_translation(p)
        value.set_rotation(q)
        self.assertTrue(np.allclose(
            value.get_isometry().matrix(), X_expected.matrix()))
        # - Ensure ordering is ((px, py, pz), (qw, qx, qy, qz))
        vector_expected = np.hstack((p, q.wxyz()))
        vector_actual = value.get_value()
        self.assertTrue(np.allclose(vector_actual, vector_expected))


if __name__ == "__main__":
    unittest.main()
