import copy
import unittest

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
        for size in range(1, 7):
            vec = [1.0] * size
            for key in ["Deterministic", "Gaussian", "Uniform"]:
                name = f"{key}Vector{size}"
                with self.subTest(name=name):
                    dut_cls = getattr(mut, name)
                    if key == "Deterministic":
                        init_args = [vec]
                    else:
                        init_args = [vec, vec]
                    dut = dut_cls(*init_args)
                    self._check_distribution_vector(dut)

    def test_distribution_vector_variant(self):
        """Confirms that the free functions that operate on a vector variant
        are bound."""
        items = [
            mut.DeterministicVectorX(value=[1.0]),
            mut.GaussianVectorX(mean=[1.0], stddev=[0.1]),
            mut.UniformVectorX(min=[-1.0], max=[1.0]),
            mut.DeterministicVector3(value=[1.0]*3),
            mut.GaussianVector3(mean=[1.0]*3, stddev=[0.1]*3),
            mut.UniformVector3(min=[-1.0]*3, max=[1.0]*3),
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
        copy.copy(dut)
        copy.deepcopy(dut)
        dut.IsDeterministic()
        dut.GetDeterministicValue()
        dut.ToSymbolic()
        dut.set_rpy_deg([0.1, 0.2, 0.3])

    def test_transform(self):
        mut.Transform()
        dut = mut.Transform(pydrake.math.RigidTransform())
        copy.copy(dut)
        copy.deepcopy(dut)
        dut.set_rotation_rpy_deg([0.1, 0.2, 0.3])
        dut.IsDeterministic()
        dut.GetDeterministicValue()
        dut.ToSymbolic()
        dut.Sample(generator=RandomGenerator())
        dut.base_frame = "name"
