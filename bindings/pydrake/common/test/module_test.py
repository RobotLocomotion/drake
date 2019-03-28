from __future__ import absolute_import, division, print_function

import os
import unittest

import six
from six import text_type as unicode

import pydrake.common


class TestCommon(unittest.TestCase):
    if six.PY2:
        assertRegex = unittest.TestCase.assertRegexpMatches

    def test_drake_demand_throws(self):
        # Drake's assertion errors should turn into SystemExit by default,
        # without the user needing to do anything special.  Here, we trigger a
        # C++ assertion failure from Python and confirm that an exception with
        # an appropriate type and message comes out.
        try:
            pydrake.common.trigger_an_assertion_failure()
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
        pydrake.common.FindResourceOrThrow(
            'drake/examples/atlas/urdf/atlas_convex_hull.urdf'
            )

    def test_temp_directory(self):
        self.assertEqual(os.environ.get('TEST_TMPDIR'),
                         pydrake.common.temp_directory())

    def test_random_distribution(self):
        # Simply test the spelling
        pydrake.common.RandomDistribution.kUniform
        pydrake.common.RandomDistribution.kGaussian
        pydrake.common.RandomDistribution.kExponential

    def test_logging(self):
        self.assertTrue(pydrake.common._module_py._HAVE_SPDLOG)
        self.assertIsInstance(
            pydrake.common.set_log_level(level="unchanged"), unicode)

    def test_random_generator(self):
        g1 = pydrake.common.RandomGenerator()
        self.assertEqual(g1(), 3499211612)
        g2 = pydrake.common.RandomGenerator(10)
        self.assertEqual(g2(), 3312796937)
