import dataclasses as dc
import math
from textwrap import dedent
import typing
import unittest

import numpy as np

import pydrake.common.schema as mut
from pydrake.common.yaml import yaml_load_typed
from pydrake.math import RollPitchYaw


@dc.dataclass
class StochasticSingular:
    deterministic: typing.Optional[mut.Deterministic] = None
    gaussian: typing.Optional[mut.Gaussian] = None
    uniform: typing.Optional[mut.Uniform] = None
    discrete: typing.Optional[mut.UniformDiscrete] = None


@dc.dataclass
class StochasticVectors:
    deterministic: typing.Optional[mut.DeterministicVectorX] = None
    gaussian: typing.Optional[mut.GaussianVectorX] = None
    uniform: typing.Optional[mut.UniformVectorX] = None


class TestStochasticSerialization(unittest.TestCase):
    """Serialization tests related to schema/stochastic.h"""

    def test_singular_types(self):
        data = dedent("""
        deterministic: { value: 5.0 }
        gaussian: { mean: 2.0, stddev: 4.0 }
        uniform: { min: 1.0, max: 5.0 }
        discrete: { values: [1, 1.5, 2] }
        """)
        x = yaml_load_typed(schema=StochasticSingular, data=data)
        self.assertEqual(x.deterministic.value, 5.0)
        self.assertEqual(x.gaussian.mean, 2.0)
        self.assertEqual(x.gaussian.stddev, 4.0)
        self.assertEqual(x.uniform.min, 1.0)
        self.assertEqual(x.uniform.max, 5.0)
        self.assertEqual(x.discrete.values, [1, 1.5, 2])

    def test_vector_types(self):
        data = dedent("""
        deterministic: { value: [3.0, 4.0, 5.0] }
        gaussian: { mean: [2.1, 2.2, 2.3, 2.4], stddev: [1.0] }
        uniform: { min: [10, 20], max: [11, 22] }
        """)
        x = yaml_load_typed(schema=StochasticVectors, data=data)
        self.assertEqual(len(x.deterministic.value), 3)
        self.assertEqual(len(x.gaussian.mean), 4)
        self.assertEqual(len(x.gaussian.stddev), 1)
        self.assertEqual(len(x.uniform.min), 2.0)
        self.assertEqual(len(x.uniform.max), 2.0)


class TestRotationSerialization(unittest.TestCase):
    """Serialization tests related to schema/rotation.h"""

    def test_rpy(self):
        data = "value: !Rpy { deg: [10.0, 20.0, 30.0] }"
        x = yaml_load_typed(schema=mut.Rotation, data=data)
        self.assertTrue(x.IsDeterministic())
        rpy = RollPitchYaw(x.GetDeterministicValue())
        rpy_deg = np.array([math.degrees(z) for z in rpy.vector()])
        expected = np.array([10.0, 20.0, 30.0])
        np.testing.assert_allclose(rpy_deg, expected)

    def test_angle_axis(self):
        data = "value: !AngleAxis { angle_deg: 10.0, axis: [0, 1, 0] }"
        x = yaml_load_typed(schema=mut.Rotation, data=data)
        self.assertTrue(x.IsDeterministic())
        aa = x.GetDeterministicValue().ToAngleAxis()
        self.assertAlmostEqual(math.degrees(aa.angle()), 10.0)
        np.testing.assert_allclose(aa.axis(), [0.0, 1.0, 0.0])

    def test_uniform(self):
        data = "value: !Uniform {}"
        x = yaml_load_typed(schema=mut.Rotation, data=data)
        self.assertFalse(x.IsDeterministic())
        x.ToSymbolic()

    def test_rpy_uniform(self):
        data = dedent("""
        value: !Rpy
          deg: !UniformVector
            min: [0, 10, 20]
            max: [30, 40, 50]
        """)
        x = yaml_load_typed(schema=mut.Rotation, data=data)
        self.assertFalse(x.IsDeterministic())


# TODO(jwnimmer-tri) Add serialization tests for schema.Translation. For now,
# the weird C++ `MakeNameValue("rotation", &rotation.value)` is incompatible
# with how our Python deserialization works.
