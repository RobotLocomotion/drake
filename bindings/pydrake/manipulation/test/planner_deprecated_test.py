from pydrake.common.test_utilities.deprecation import catch_drake_warnings

import unittest


# 2023-06-01 Delete this file with the rest of the deprecation.

class TestDeprecation(unittest.TestCase):
    def test_deprecated_symbols_exist(self):
        # The whole point of this test is that we can access the same symbols
        # tested in
        # /bindings/pydrake/multibody/test/inverse_kinematics_differential_test.py  # noqa
        # via the old, deprecated module path. We should get a single
        # deprecation warning on the module. Otherwise, all of the given
        # symbols should be successfully imported.
        with catch_drake_warnings(expected_count=1) as w:
            from pydrake.manipulation.planner import (
                DifferentialInverseKinematicsIntegrator,
                DifferentialInverseKinematicsStatus,
                DifferentialInverseKinematicsParameters,
                DifferentialInverseKinematicsResult,
                DoDifferentialInverseKinematics,
            )
            self.assertIn("2023-06-01", str(w[0].message))
