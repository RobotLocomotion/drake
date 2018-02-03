# -*- coding: utf8 -*-

from pydrake.systems.rendering import (
    PoseVector,
)

import copy
import unittest
import numpy as np

from pydrake.systems.framework import (
    BasicVector,
)


def tform(X_AB, p_B):
    # Returns p_B transformed by X_AB (p_A).
    # TODO(eric.cousineau): Find some good geometry utilities?
    p_B_homogeneous = np.hstack((p_B, 1))
    return np.dot(X_AB, p_B_homogeneous)[:-1]


class TestRendering(unittest.TestCase):
    def test_pose_vector(self):
        # Assume this pose represents `X_WA`.
        value = PoseVector()
        self.assertTrue(isinstance(value, BasicVector))
        self.assertTrue(isinstance(copy.copy(value), PoseVector))
        self.assertTrue(isinstance(value.Clone(), PoseVector))
        self.assertEquals(value.size(), PoseVector.kSize)
        self.assertTrue(np.allclose(value.get_isometry(), np.eye(4, 4)))
        # Use values from `pose_vector_test`.
        # - Translation.
        value = PoseVector()
        p_A = [1, 2, 3]
        value.set_translation(p_A)
        self.assertTrue(np.allclose(value.get_translation(), p_A))
        p_W = tform(value.get_isometry(), p_A)
        self.assertTrue(np.allclose(p_W, [2, 4, 6]))
        # - Orientation.
        value = PoseVector()
        q_WA = [0.5, 0.5, 0.5, 0.5]
        p_A = [1, 0, 0]
        value.set_rotation(q_WA)
        self.assertTrue(np.allclose(value.get_rotation(), q_WA))
        p_W = tform(value.get_isometry(), p_A)
        self.assertTrue(np.allclose(p_W, [0, 1, 0]))
        # - Ordering.
        value = PoseVector()
        p_WA = [0, 1, 2]  # (x, y, z)
        q_WA = [0.1, 0.3, 0.7, 0.9]  # (w, x, y, z)
        q_WA = q_WA / np.linalg.norm(q_WA)
        value.set_translation(p_WA)
        value.set_rotation(q_WA)
        vector_expected = np.hstack((p_WA, q_WA))
        vector_actual = value.get_value()
        self.assertTrue(np.allclose(value.get_value(), vector_expected))


if __name__ == "__main__":
    unittest.main()
