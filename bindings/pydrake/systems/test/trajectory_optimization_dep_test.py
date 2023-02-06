from pydrake.common.test_utilities.deprecation import catch_drake_warnings

import unittest


class TestDeprecation(unittest.TestCase):
    def test_deprecated_symbols_exist(self):
        # The whole point of this test is that we can access the same symbols
        # tested in
        #  /bindings/pydrake/planning/test/trajectory_optimization_test.py via
        # the old, deprecated module path. We should get a single deprecation
        # warning on the module. Otherwise, all of the given symbols should be
        # successfully imported.
        with catch_drake_warnings(expected_count=1) as w:
            from pydrake.systems.trajectory_optimization import (
                AddDirectCollocationConstraint,
                DirectCollocation,
                DirectCollocationConstraint,
                DirectTranscription,
                KinematicTrajectoryOptimization,
                TimeStep,
            )
            self.assertIn("2023-05-01", str(w[0].message))
