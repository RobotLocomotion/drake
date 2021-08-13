import unittest
import warnings

import pydrake.lcmtypes

# TODO(jwnimmer-tri) When deprecated drake.lcmt_foo is removed on or after
# 2021-12-01, we should remove this entire file.


class AcceptanceTest(unittest.TestCase):

    def test_deprecated_names(self):
        # Confirm that the deprecated names exist, under a deprecation warning.
        with warnings.catch_warnings(record=True) as caught:
            from drake import (
                lcmt_drake_signal,
                lcmt_robot_plan,
                lcmt_robot_state)
        self.assertEqual(len(caught), 1)
        self.assertIn("pydrake.lcmtypes.lcmt_foo instead of drake.lcmt_foo",
                      str(caught[0]))

        # Confirm that the deprecated names are aliases to the new names.
        self.assertEqual(lcmt_drake_signal, pydrake.lcmtypes.lcmt_drake_signal)
        self.assertEqual(lcmt_robot_plan, pydrake.lcmtypes.lcmt_robot_plan)
        self.assertEqual(lcmt_robot_state, pydrake.lcmtypes.lcmt_robot_state)

        # Sanity check the deprecated types.
        outer = lcmt_robot_plan()
        outer.num_states = 1
        outer.plan.append(lcmt_robot_state())
        outer.encode()
