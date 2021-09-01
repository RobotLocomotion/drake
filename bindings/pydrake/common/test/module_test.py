import os
import unittest

import numpy as np

from pydrake.autodiffutils import AutoDiffXd
import pydrake.common as mut
import pydrake.common._module_py._testing as mut_testing
from pydrake.common.test_utilities.deprecation import catch_drake_warnings


class TestCommon(unittest.TestCase):
    def test_drake_demand_throws(self):
        # Drake's assertion errors should turn into SystemExit by default,
        # without the user needing to do anything special.  Here, we trigger a
        # C++ assertion failure from Python and confirm that an exception with
        # an appropriate type and message comes out.
        try:
            mut.trigger_an_assertion_failure()
            self.fail("Did not get a SystemExit")
        except SystemExit as e:
            self.assertTrue(e.code is not None)
            self.assertRegex(
                str(e),
                ".*".join([
                    "Failure at ",
                    " trigger_an_assertion_failure",
                    " condition 'false' failed",
                ]))

    def test_find_resource_or_throw(self):
        mut.FindResourceOrThrow(
            'drake/examples/atlas/urdf/atlas_convex_hull.urdf'
            )

    def test_test_temp_directory(self):
        temp_dir = mut.temp_directory()
        # We'll simply confirm that the path *starts* with the TEST_TMPDIR and
        # that it exists. We'll assume that it otherwise has the documented
        # properties.
        self.assertTrue(temp_dir.startswith(os.environ.get('TEST_TMPDIR')))
        self.assertTrue(os.path.exists(temp_dir))

    def test_tolerance_type(self):
        # Simply test the spelling
        mut.ToleranceType.kAbsolute
        mut.ToleranceType.kRelative

    def test_random_distribution(self):
        # Simply test the spelling
        mut.RandomDistribution.kUniform
        mut.RandomDistribution.kGaussian
        mut.RandomDistribution.kExponential

    def test_logging(self):
        self.assertTrue(mut._module_py._HAVE_SPDLOG)
        self.assertIsInstance(
            mut.set_log_level(level="unchanged"), str)

    def test_random_generator(self):
        g1 = mut.RandomGenerator()
        self.assertEqual(g1(), 3499211612)
        g2 = mut.RandomGenerator(10)
        self.assertEqual(g2(), 3312796937)

    def test_random_numpy_coordination(self):
        # Verify that multiple numpy generators can be seeded from
        # a single RandomGenerator without duplicating values (as
        # discussed in #12632).
        g = mut.RandomGenerator(123)
        rs1 = np.random.RandomState(seed=g())
        rs1.randint(100)  # discard one
        rs2 = np.random.RandomState(seed=g())
        self.assertNotEqual(rs1.randint(100), rs2.randint(100))

    def test_calc_probability_density(self):
        density_val = mut.CalcProbabilityDensity(
            distribution=mut.RandomDistribution.kGaussian,
            x=np.array([0.5, 1.]))
        density_ad = mut.CalcProbabilityDensity(
            distribution=mut.RandomDistribution.kGaussian,
            x=np.array([AutoDiffXd(1), AutoDiffXd(2)]))

    def test_assert_is_armed(self):
        self.assertIsInstance(mut.kDrakeAssertIsArmed, bool)

    def test_nice_type_name(self):
        """Tests behavior of ``PyNiceTypeNamePtrOverride`` in module_py.cc."""
        obj = mut_testing.RegisteredType()
        registered_type_py_name = f"{mut_testing.__name__}.RegisteredType"
        registered_type_cc_name = (
            "drake::pydrake::(anonymous)::testing::RegisteredType")
        unregistered_type_cc_name = (
            "drake::pydrake::(anonymous)::testing::UnregisteredType")
        unregistered_derived_type_cc_name = (
            "drake::pydrake::(anonymous)::testing::UnregisteredDerivedType")
        # Type and instance are registered with Python, so it should return the
        # Python type name.
        self.assertEqual(
            mut_testing.get_nice_type_name_cc_registered_instance(obj),
            registered_type_py_name)
        # Type is known, but instance is unregistered, so it should return the
        # C++ type name.
        self.assertEqual(
            mut_testing.get_nice_type_name_cc_unregistered_instance(),
            registered_type_cc_name)
        # Uses raw typeid for a registered type, so it should return the C++
        # type name.
        self.assertEqual(
            mut_testing.get_nice_type_name_cc_typeid(obj),
            registered_type_cc_name)
        # Type and instance are unregistered, so it should return the C++ type
        # name.
        self.assertEqual(
            mut_testing.get_nice_type_name_cc_unregistered_type(),
            unregistered_type_cc_name)
        # Type is unregistered but derived from a registered base type (to
        # mimic Value<> / AbstractValue), and instance is registered. Should
        # return C++ type name.
        base_only_instance = mut_testing.make_cc_unregistered_derived_type()
        self.assertIs(type(base_only_instance), mut_testing.RegisteredType)
        self.assertEqual(
            mut_testing.get_nice_type_name_cc_registered_instance(
                base_only_instance),
            unregistered_derived_type_cc_name)
