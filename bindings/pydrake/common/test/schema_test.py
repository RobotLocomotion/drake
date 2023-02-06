import copy
import unittest

import numpy as np

from pydrake.common import RandomGenerator
import pydrake.common.schema as mut
import pydrake.math


class TestSchema(unittest.TestCase):

    def _check_distribution(self, dut):
        """Confirms that a subclass instance of Distribution
        * binds the base methods, and
        * supports copy/deepcopy (even though the superclass does not).
        """
        self.assertIsInstance(dut, mut.Distribution)
        copy.copy(dut)
        copy.deepcopy(dut)
        dut.Sample(generator=RandomGenerator())
        dut.Mean()
        dut.ToSymbolic()

    def test_deterministic(self):
        mut.Deterministic()
        mut.Deterministic(0.5)
        dut = mut.Deterministic(value=1.0)
        dut.value = 2.0
        self._check_distribution(dut)

    def test_gaussian(self):
        mut.Gaussian()
        mut.Gaussian(0.5, 0.2)
        dut = mut.Gaussian(mean=1.0, stddev=0.1)
        dut.mean = 2.0
        dut.stddev = 0.2
        self._check_distribution(dut)

    def test_uniform(self):
        mut.Uniform()
        mut.Uniform(-0.5, 0.5)
        dut = mut.Uniform(min=-1.0, max=1.0)
        dut.min = -2.0
        dut.max = 2.0
        self._check_distribution(dut)

    def test_uniform_discrete(self):
        mut.UniformDiscrete()
        mut.UniformDiscrete([0.0, 1.0])
        dut = mut.UniformDiscrete(values=[0.0, 0.1])
        dut.values = [0.0, 0.2]
        self._check_distribution(dut)

    def test_distribution_variant(self):
        """Confirms that the free functions that operate on a variant are
        bound."""
        items = [
            mut.Deterministic(1.0),
            mut.Gaussian(1.0, 0.1),
            mut.Uniform(-1.0, 1.0),
            mut.UniformDiscrete([0.0, 1.0]),
        ]
        for item in items:
            copied = mut.ToDistribution(item)
            self._check_distribution(copied)
            mut.Sample(var=item, generator=RandomGenerator())
            mut.Mean(var=item)
            mut.ToSymbolic(var=item)
            if mut.IsDeterministic(var=item):
                mut.GetDeterministicValue(var=item)

    def _check_distribution_vector(self, dut):
        """Confirms that a subclass instance of DistributionVector
        * binds the base methods, and
        * supports copy/deepcopy (even though the superclass does not).
        """
        self.assertIsInstance(dut, mut.DistributionVector)
        copy.copy(dut)
        copy.deepcopy(dut)
        dut.Sample(generator=RandomGenerator())
        dut.Mean()
        dut.ToSymbolic()

    def test_deterministic_vector(self):
        mut.DeterministicVectorX()
        mut.DeterministicVectorX([0.1, 0.2])
        dut = mut.DeterministicVectorX(value=[0.0, 1.0])
        dut.value = [0.0, 2.0]
        self._check_distribution_vector(dut)

    def test_gaussian_vector(self):
        mut.GaussianVectorX()
        mut.GaussianVectorX([-0.5, 0.5], [0.2, 0.2])
        dut = mut.GaussianVectorX(mean=[-1.0, 1.0], stddev=[0.1, 0.1])
        dut.mean = [-2.0, 2.0]
        dut.stddev = [0.2, 0.2]
        self._check_distribution_vector(dut)

    def test_uniform_vector(self):
        mut.UniformVectorX()
        mut.UniformVectorX([-0.5, -5.0], [0.5, 5.0])
        dut = mut.UniformVectorX(min=[-1.0, -10.0], max=[1.0, 10.0])
        dut.min = [-2.0, -20.0]
        dut.max = [2.0, 20.0]
        self._check_distribution_vector(dut)

    def test_sized_vectors(self):
        """Spot check the fixed-size stochastic vectors."""
        for size in [None, 1, 2, 3, 4, 5, 6]:
            vec_data = [1.0] * (size or 3)
            for template in [mut.DeterministicVector,
                             mut.GaussianVector,
                             mut.UniformVector]:
                with self.subTest(template=template, size=size):
                    dut_cls = template[size]
                    if template == mut.DeterministicVector:
                        init_args = [vec_data]
                    else:
                        init_args = [vec_data, vec_data]
                    dut = dut_cls(*init_args)
                    self._check_distribution_vector(dut)

    def test_distribution_vector_variant(self):
        """Confirms that the free functions that operate on a vector variant
        are bound."""
        items = [
            mut.DeterministicVectorX(value=[1.0]),
            mut.GaussianVectorX(mean=[1.0], stddev=[0.1]),
            mut.UniformVectorX(min=[-1.0], max=[1.0]),
            mut.DeterministicVector[3](value=[1.0]*3),
            mut.GaussianVector[3](mean=[1.0]*3, stddev=[0.1]*3),
            mut.UniformVector[3](min=[-1.0]*3, max=[1.0]*3),
        ]
        for item in items:
            copied = mut.ToDistributionVector(item)
            self._check_distribution_vector(copied)
            if mut.IsDeterministic(vec=item):
                mut.GetDeterministicValue(vec=item)

    def test_rotation(self):
        for dut in [mut.Rotation(),
                    mut.Rotation(pydrake.math.RotationMatrix()),
                    mut.Rotation(pydrake.math.RollPitchYaw([0.0, 0.0, 0.0]))]:
            # The dut should be the identity.
            self.assertTrue(dut.IsDeterministic())
            rotmat = dut.GetDeterministicValue()
            self.assertTrue(rotmat.IsExactlyIdentity())

            # All getter functions work without crashing.
            dut.ToSymbolic()

            # The class is copyable and has a real repr.
            mut.Rotation(other=dut)
            copy.copy(dut)
            copy.deepcopy(dut)
            self.assertIn("value", repr(dut))

        # Setters.
        dut.set_rpy_deg([0.1, 0.2, 0.3])
        np.testing.assert_equal(dut.value.deg, [0.1, 0.2, 0.3])

        # Attributes.
        self.assertIsInstance(
            mut.Rotation(value=mut.Rotation.AngleAxis()).value,
            mut.Rotation.AngleAxis)

    def test_rotation_nested_classes(self):
        # The class is copyable and has a real repr.
        for dut_cls in [mut.Rotation.Identity,
                        mut.Rotation.Uniform,
                        mut.Rotation.Rpy,
                        mut.Rotation.AngleAxis]:
            dut = dut_cls()
            dut_cls(other=dut)
            copy.copy(dut)
            copy.deepcopy(dut)
            self.assertNotIn("0x", repr(dut))

        # Properties are bound.
        np.testing.assert_equal(mut.Rotation.Rpy(deg=[1, 2, 3]).deg, [1, 2, 3])
        self.assertEqual(mut.Rotation.AngleAxis(angle_deg=5).angle_deg, 5)

    def test_transform(self):
        for dut in [mut.Transform(),
                    mut.Transform(pydrake.math.RigidTransform())]:
            # The dut should be the identity.
            self.assertIsNone(dut.base_frame)
            self.assertTrue(dut.IsDeterministic())
            np.testing.assert_equal(dut.translation, [0, 0, 0])
            rotmat = dut.rotation.GetDeterministicValue()
            self.assertTrue(rotmat.IsExactlyIdentity())

            # All getter functions work without crashing.
            dut.ToSymbolic()
            dut.Mean()
            dut.Sample(generator=RandomGenerator())

            # The class is copyable and has a real repr.
            mut.Transform(other=dut)
            copy.copy(dut)
            copy.deepcopy(dut)
            self.assertIn("base_frame", repr(dut))

        # Setters.
        dut.base_frame = "name"
        self.assertEqual(dut.base_frame, "name")

        dut.translation = [1.0, 2.0, 3.0]
        np.testing.assert_equal(dut.translation, [1.0, 2.0, 3.0])

        dut.set_rotation_rpy_deg([0.1, 0.2, 0.3])
        np.testing.assert_equal(dut.rotation.value.deg, [0.1, 0.2, 0.3])
        dut.rotation.value.deg = [0.4, 0.5, 0.6]
        np.testing.assert_equal(dut.rotation.value.deg, [0.4, 0.5, 0.6])

        # Attributes.
        self.assertEqual(mut.Transform(base_frame="base").base_frame, "base")
