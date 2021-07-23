import unittest
import numpy as np
from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.gurobi import GurobiSolver
from pydrake.common import temp_directory
import os


class TestMathematicalProgram(unittest.TestCase):
    def test_gurobi_solver(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint(x[0] >= 1)
        prog.AddLinearConstraint(x[1] >= 1)
        prog.AddQuadraticCost(np.eye(2), np.zeros(2), x)
        solver = GurobiSolver()
        self.assertEqual(solver.solver_id(), GurobiSolver.id())
        self.assertTrue(solver.available())
        self.assertEqual(solver.solver_type(), mp.SolverType.kGurobi)
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        x_expected = np.array([1, 1])
        self.assertTrue(np.allclose(result.GetSolution(x), x_expected))
        self.assertGreater(result.get_solver_details().optimizer_time, 0.)
        self.assertEqual(result.get_solver_details().error_code, 0)
        self.assertEqual(result.get_solver_details().optimization_status, 2)
        self.assertTrue(np.isnan(result.get_solver_details().objective_bound))

    def test_gurobi_socp_dual(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        constraint = prog.AddLorentzConeConstraint([2., 2*x[0], 3 * x[1] + 1])
        prog.AddLinearCost(x[1])
        solver = GurobiSolver()
        options = mp.SolverOptions()
        options.SetOption(solver.solver_id(), "QCPDual", 1)
        result = solver.Solve(prog, None, options)
        np.testing.assert_allclose(
            result.GetDualSolution(constraint), np.array([-1./12]), atol=1e-7)

    def test_gurobi_license(self):
        # Nominal use case.
        with GurobiSolver.AcquireLicense():
            pass
        # Inspect.
        with GurobiSolver.AcquireLicense() as license:
            self.assertTrue(license.is_valid())
        self.assertFalse(license.is_valid())

    def test_write_to_file(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2)
        prog.AddLinearConstraint(x[0] + x[1] == 1)
        prog.AddQuadraticCost(x[0] * x[0] + x[1] * x[1])
        solver = GurobiSolver()
        file_name = temp_directory() + "/gurobi.mps"
        options = mp.SolverOptions()
        options.SetOption(solver.id(), "GRBwrite", file_name)
        result = solver.Solve(prog, None, options)
        self.assertTrue(os.path.exists(file_name))

    def test_compute_iis(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2)
        prog.AddBoundingBoxConstraint(1, np.inf, x)
        prog.AddLinearConstraint(x[0] + x[1] == 1)
        solver = GurobiSolver()
        ilp_file_name = temp_directory() + "/gurobi.ilp"
        options = mp.SolverOptions()
        options.SetOption(solver.id(), "GRBwrite", ilp_file_name)
        options.SetOption(solver.id(), "GRBcomputeIIS", 1)
        result = solver.Solve(prog, None, options)
        self.assertTrue(os.path.exists(ilp_file_name))
