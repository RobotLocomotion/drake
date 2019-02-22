import unittest

import numpy as np

from pydrake.multibody.math import (
    SpatialAcceleration,
    SpatialVelocity,
)


class TestMultibodyTreeMath(unittest.TestCase):
    def check_spatial_vector(self, cls, rotation_name, translation_name):
        vec = cls()
        # - Accessors.
        self.assertTrue(isinstance(vec.rotational(), np.ndarray))
        self.assertTrue(isinstance(vec.translational(), np.ndarray))
        self.assertEqual(vec.rotational().shape, (3,))
        self.assertEqual(vec.translational().shape, (3,))
        # - Fully-parameterized constructor.
        rotation_expected = [0.1, 0.3, 0.5]
        translation_expected = [0., 1., 2.]
        kwargs = {
            rotation_name: rotation_expected,
            translation_name: translation_expected,
        }
        vec1 = cls(**kwargs)
        self.assertTrue(np.allclose(vec1.rotational(), rotation_expected))
        self.assertTrue(
            np.allclose(vec1.translational(), translation_expected))

    def test_spatial_vector_types(self):
        self.check_spatial_vector(SpatialVelocity, 'w', 'v')
        self.check_spatial_vector(SpatialAcceleration, 'alpha', 'a')
