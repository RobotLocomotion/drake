import os
import unittest

import pydrake.common as mut


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

    def test_temp_directory(self):
        self.assertEqual(os.environ.get('TEST_TMPDIR'),
                         mut.temp_directory())

    def test_tolerance_type(self):
        # Simply test the spelling
        mut.ToleranceType.absolute
        mut.ToleranceType.relative

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

    def test_assert_is_armed(self):
        self.assertIsInstance(mut.kDrakeAssertIsArmed, bool)
