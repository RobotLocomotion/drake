import unittest
import numpy as np
from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.mosek import MosekSolver


class TestMathematicalProgram(unittest.TestCase):
    def test_mosek_solver(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint(x[0] >= 1)
        prog.AddLinearConstraint(x[1] >= 1)
        prog.AddQuadraticCost(np.eye(2), np.zeros(2), x)
        solver = MosekSolver()
        self.assertEqual(solver.solver_id(), MosekSolver.id())
        # Mosek prints output to the terminal.
        solver_options = mp.SolverOptions()
        solver_options.SetOption(mp.CommonSolverOption.kPrintToConsole, 1)
        self.assertTrue(solver.available())
        self.assertEqual(solver.solver_type(), mp.SolverType.kMosek)
        result = solver.Solve(prog, None, solver_options)
        self.assertTrue(result.is_success())
        x_expected = np.array([1, 1])
        self.assertTrue(np.allclose(result.GetSolution(x), x_expected))
        self.assertEqual(result.get_solver_details().solution_status, 1)
        self.assertEqual(result.get_solver_details().rescode, 0)
        self.assertGreater(result.get_solver_details().optimizer_time, 0.)

    def test_mosek_license(self):
        # Nominal use case.
        with MosekSolver.AcquireLicense():
            pass
        # Inspect.
        with MosekSolver.AcquireLicense() as license:
            self.assertTrue(license.is_valid())
        self.assertFalse(license.is_valid())
