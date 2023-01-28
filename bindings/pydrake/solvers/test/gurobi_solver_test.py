import os
import unittest

import numpy as np

from pydrake.common import temp_directory
from pydrake.solvers import (
    GurobiSolver,
    MathematicalProgram,
    SolverOptions,
    SolverType,
)


class TestMathematicalProgram(unittest.TestCase):
    def test_gurobi_solver(self):
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint(x[0] >= 1)
        prog.AddLinearConstraint(x[1] >= 1)
        prog.AddQuadraticCost(np.eye(2), np.zeros(2), x)
        solver = GurobiSolver()
        self.assertEqual(solver.solver_id(), GurobiSolver.id())
        self.assertTrue(solver.available())
        self.assertEqual(solver.solver_type(), SolverType.kGurobi)
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        x_expected = np.array([1, 1])
        self.assertTrue(np.allclose(result.GetSolution(x), x_expected))
        self.assertGreater(result.get_solver_details().optimizer_time, 0.)
        self.assertEqual(result.get_solver_details().error_code, 0)
        self.assertEqual(result.get_solver_details().optimization_status, 2)
        self.assertTrue(np.isnan(result.get_solver_details().objective_bound))

    def test_gurobi_socp_dual(self):
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        constraint = prog.AddLorentzConeConstraint([2., 2*x[0], 3 * x[1] + 1])
        prog.AddLinearCost(x[1])
        solver = GurobiSolver()
        options = SolverOptions()
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
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(2)
        prog.AddLinearConstraint(x[0] + x[1] == 1)
        prog.AddQuadraticCost(x[0] * x[0] + x[1] * x[1])
        solver = GurobiSolver()
        file_name = temp_directory() + "/gurobi.mps"
        options = SolverOptions()
        options.SetOption(solver.id(), "GRBwrite", file_name)
        result = solver.Solve(prog, None, options)
        self.assertTrue(os.path.exists(file_name))

    def test_compute_iis(self):
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(2)
        prog.AddBoundingBoxConstraint(1, np.inf, x)
        prog.AddLinearConstraint(x[0] + x[1] == 1)
        solver = GurobiSolver()
        ilp_file_name = temp_directory() + "/gurobi.ilp"
        options = SolverOptions()
        options.SetOption(solver.id(), "GRBwrite", ilp_file_name)
        options.SetOption(solver.id(), "GRBcomputeIIS", 1)
        result = solver.Solve(prog, None, options)
        self.assertTrue(os.path.exists(ilp_file_name))

    def test_callback(self):
        prog = MathematicalProgram()
        b = prog.NewBinaryVariables(4)
        prog.AddLinearConstraint(b[0] <= 1 - 0.5 * b[1])
        prog.AddLinearConstraint(b[1] <= 1 - 0.5 * b[0])
        prog.AddLinearCost(-b[0] - b[1])

        prog.SetSolverOption(GurobiSolver.id(), "Presolve", 0)
        prog.SetSolverOption(GurobiSolver.id(), "Heuristics", 0.)
        prog.SetSolverOption(GurobiSolver.id(), "Cuts", 0)
        prog.SetSolverOption(GurobiSolver.id(), "NodeMethod", 2)

        b_init = np.array([0, 0., 0., 0.])

        prog.SetInitialGuess(b, b_init)
        solver = GurobiSolver()

        explored_node_count = 0

        def node_callback(prog, solver_status_info, x, x_vals):
            nonlocal explored_node_count
            explored_node_count = solver_status_info.explored_node_count

        solver.AddMipNodeCallback(
            callback=lambda prog, solver_status_info, x, x_vals: node_callback(
                prog, solver_status_info, x, x_vals))

        best_objectives = []

        def sol_callback(prog, callback_info, objectives):
            print(f"explored nodes {callback_info.explored_node_count}")
            objectives.append(callback_info.best_objective)

        solver.AddMipSolCallback(
            callback=lambda prog, callback_info: sol_callback(
                prog, callback_info, best_objectives))

        result = solver.Solve(prog)
        self.assertTrue(result.is_success())
        self.assertGreater(explored_node_count, 0)
        self.assertGreater(len(best_objectives), 0)
