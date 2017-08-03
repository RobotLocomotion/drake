from __future__ import absolute_import, division, print_function

import unittest
import pydrake.common


class TestCommon(unittest.TestCase):
    def testDrakeDemandThrows(self):
        # Drake's assertion errors should turn into SystemExit by default,
        # without the user needing to do anything special.  Here, we trigger a
        # C++ assertion failure from Python and confirm that an exception with
        # an appropriate type and message comes out.
        try:
            pydrake.common.trigger_an_assertion_failure()
            self.fail("Did not get a SystemExit")
        except SystemExit as e:
            self.assertTrue(e.code is not None)
            self.assertRegexpMatches(
                e.message,
                ".*".join([
                    "Failure at ",
                    " trigger_an_assertion_failure",
                    " condition 'false' failed",
                ]))

    def testDrakeFindResourceOrThrow(self):
        pydrake.common.FindResourceOrThrow(
            'drake/examples/atlas/urdf/atlas_convex_hull.urdf'
            )


if __name__ == '__main__':
    unittest.main()
