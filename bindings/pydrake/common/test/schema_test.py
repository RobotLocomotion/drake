import unittest

from pydrake.common import RandomGenerator
import pydrake.common.schema as mut
import pydrake.math


class TestSchema(unittest.TestCase):

    def _check_distribution(self, dut):
        """Confirms that subclasses of Distribution bind the base methods."""
        self.assertIsInstance(dut, mut.Distribution)
        dut.Sample(generator=RandomGenerator())
        dut.Mean()
        dut.ToSymbolic()

    def test_deterministic(self):
        mut.Deterministic()
        dut = mut.Deterministic(1.0)
        dut.value = 2.0
        self._check_distribution(dut)

    def test_gaussian(self):
        mut.Gaussian()
        dut = mut.Gaussian(mean=1.0, stddev=0.1)
        dut.mean = 2.0
        dut.stddev = 0.2
        self._check_distribution(dut)

    def test_uniform(self):
        mut.Uniform()
        dut = mut.Uniform(min=-1.0, max=1.0)
        dut.min = -2.0
        dut.max = 2.0
        self._check_distribution(dut)

    def test_uniform_discrete(self):
        mut.UniformDiscrete()
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
        """Confirms that subclasses of DistributionVector bind the base
        methods.
        """
        self.assertIsInstance(dut, mut.DistributionVector)
        dut.Sample(generator=RandomGenerator())
        dut.Mean()
        dut.ToSymbolic()

    def test_deterministic_vector(self):
        mut.DeterministicVectorX()
        dut = mut.DeterministicVectorX(value=[0.0, 1.0])
        dut.value = [0.0, 2.0]
        self._check_distribution_vector(dut)

    def test_gaussian_vector(self):
        mut.GaussianVectorX()
        dut = mut.GaussianVectorX(mean=[-1.0, 1.0], stddev=[0.1, 0.1])
        dut.mean = [-2.0, 2.0]
        dut.stddev = [0.2, 0.2]
        self._check_distribution_vector(dut)

    def test_uniform_vector(self):
        mut.UniformVectorX()
        dut = mut.UniformVectorX(min=[-1.0, -10.0], max=[1.0, 10.0])
        dut.min = [-2.0, -20.0]
        dut.max = [2.0, 20.0]
        self._check_distribution_vector(dut)

    def test_distribution_vector_variant(self):
        """Confirms that the free functions that operate on a vector variant
        are bound."""
        items = [
            mut.DeterministicVectorX(value=[1.0]),
            mut.GaussianVectorX(mean=[1.0], stddev=[0.1]),
            mut.UniformVectorX(min=[-1.0], max=[1.0]),
        ]
        for item in items:
            copied = mut.ToDistributionVector(item)
            self._check_distribution_vector(copied)
            if mut.IsDeterministic(vec=item):
                mut.GetDeterministicValue(vec=item)

    def test_rotation(self):
        mut.Rotation()
        mut.Rotation(pydrake.math.RotationMatrix())
        dut = mut.Rotation(pydrake.math.RollPitchYaw([0.0, 0.0, 0.0]))
        dut.IsDeterministic()
        dut.GetDeterministicValue()
        dut.ToSymbolic()
        dut.set_rpy_deg([0.1, 0.2, 0.3])

    def test_transform(self):
        mut.Transform()
        dut = mut.Transform(pydrake.math.RigidTransform())
        dut.set_rotation_rpy_deg([0.1, 0.2, 0.3])
        dut.IsDeterministic()
        dut.GetDeterministicValue()
        dut.ToSymbolic()
        dut.Sample(generator=RandomGenerator())
        dut.base_frame = "name"
