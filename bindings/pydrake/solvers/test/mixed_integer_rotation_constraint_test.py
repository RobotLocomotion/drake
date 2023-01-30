import unittest

import numpy as np

from pydrake.solvers import (
    IntervalBinning,
    MathematicalProgram,
    MixedIntegerRotationConstraintGenerator,
)


class TestMixedIntegerRotationConstraint(unittest.TestCase):
    def test_MixedIntegerRotationConstraintGenerator(self):
        prog = MathematicalProgram()

        R = prog.NewContinuousVariables(3, 3, "R")
        Generator = MixedIntegerRotationConstraintGenerator
        binning = IntervalBinning.kLogarithmic
        approach = Generator.Approach.kBoth
        n_intervals = 2
        so3_generator = Generator(
            approach=approach,
            num_intervals_per_half_axis=n_intervals,
            interval_binning=binning
        )
        assert so3_generator.phi().shape == (n_intervals*2 + 1,)
        assert so3_generator.phi_nonnegative().shape == (n_intervals + 1,)
        assert so3_generator.num_intervals_per_half_axis() == n_intervals
        assert so3_generator.interval_binning() == binning

        constraint_info = so3_generator.AddToProgram(R, prog)
        assert len(constraint_info.lambda_) == 3
        assert len(constraint_info.lambda_[0]) == 3
        assert len(constraint_info.B_) == 3
        assert len(constraint_info.B_[0]) == 3
        # Check that nonzero constraints got added.
        self.assertGreater(len(prog.GetAllConstraints()), 0)
