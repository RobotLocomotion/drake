import unittest
import numpy as np
from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers import mixed_integer_optimization_util as mip_util
from pydrake.solvers import mixed_integer_rotation_constraint as mip_rot


class TestMixedIntegerRotationConstraint(unittest.TestCase):
    def test_MixedIntegerRotationConstraintGenerator(self):
        prog = mp.MathematicalProgram()

        R = prog.NewContinuousVariables(3, 3, "R")
        Generator = mip_rot.MixedIntegerRotationConstraintGenerator
        binning = mip_util.IntervalBinning.kLogarithmic
        approach = Generator.Approach.kBoth
        so3_generator = Generator(
            approach=approach,
            num_intervals_per_half_axis=2,
            interval_binning=binning
        )
        so3_generator.AddToProgram(R, prog)

        # Check that nonzero constraints got added.
        self.assertGreater(len(prog.GetAllConstraints()), 0)
