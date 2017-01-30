from __future__ import print_function, absolute_import

import unittest
import numpy as np
import pydrake
from pydrake.solvers import mathematicalprogram


class TestMathematicalProgram(unittest.TestCase):
    def test_program_construction(self):
        prog = mathematicalprogram.MathematicalProgram()
        vars = prog.NewContinuousVariables(5, "x")
        print(vars)


if __name__ == '__main__':
    unittest.main()
