import os
import unittest

from pydrake.common import temp_directory
from pydrake.solvers import (
    GenerateSDPA,
    MathematicalProgram,
)


class TestSdpaFreeFormat(unittest.TestCase):
    def test_GenerateSDPA(self):
        prog = MathematicalProgram()
        X = prog.NewSymmetricContinuousVariables(2)
        prog.AddPositiveSemidefiniteConstraint(X)
        prog.AddLinearCost(X[0, 0] + X[1, 1])
        prog.AddBoundingBoxConstraint(1, 1, X[0, 1])
        file_name = temp_directory() + "/sdpa"
        self.assertTrue(GenerateSDPA(prog=prog, file_name=file_name))
        self.assertTrue(os.path.exists(file_name + ".dat-s"))
