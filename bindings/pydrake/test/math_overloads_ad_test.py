"""
See `math_overloads_test_utilities.py` for information.
"""
import pydrake.test.math_overloads_test_utilities as mtest


class TestMathOverloads(mtest.MathOverloadsBase):
    def test_overloads(self):
        self.check_overload(mtest.AutoDiffOverloads())
