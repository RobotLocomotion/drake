import pydrake.planning as mut  # ruff: isort: skip

import unittest

import numpy as np
import scipy.sparse as sp

from pydrake.common.test_utilities import numpy_compare
from pydrake.solvers import (
    CommonSolverOption,
    GurobiSolver,
    MosekSolver,
    SolverOptions,
)


def GurobiOrMosekSolverAvailable():
    return (MosekSolver().available() and MosekSolver().enabled()) or (
        GurobiSolver().available() and GurobiSolver().enabled()
    )


class TestGraphAlgorithms(unittest.TestCase):
    """Tests the classes and methods defined in
    pydrake.planning.graph_algorithms.
    """

    def _butteryfly_graph(self):
        adjacency = np.array(
            [
                [0, 1, 1, 0, 0],
                [1, 0, 1, 0, 0],
                [1, 1, 0, 1, 1],
                [0, 0, 1, 0, 1],
                [0, 0, 1, 1, 0],
            ]
        ).astype(bool)
        data = np.ones(12).astype(bool)
        indices = [1, 2, 0, 2, 0, 1, 3, 4, 2, 4, 2, 3]
        indptr = [0, 2, 4, 8, 10, 12]
        sparse = sp.csc_matrix([data, indices, indptr], shape=(5, 5))
        assert np.all(sparse.todense() == adjacency)
        assert np.all(adjacency == adjacency.T)
        return sparse

    def test_max_clique_solver_base_subclassable(self):
        class DummyMaxCliqueSolver(mut.MaxCliqueSolverBase):
            def __init__(self, name):
                mut.MaxCliqueSolverBase.__init__(self)
                self.name = name

            def DoSolveMaxClique(self, adjacency_matrix):
                return np.ones(adjacency_matrix.shape[0])

        name = "dummy"
        solver = DummyMaxCliqueSolver(name=name)
        graph = self._butteryfly_graph()
        self.assertEqual(name, solver.name)
        numpy_compare.assert_equal(
            solver.SolveMaxClique(adjacency_matrix=graph),
            np.ones(graph.shape[0]),
        )

    def test_max_clique_solver_via_mip_methods(self):
        graph = self._butteryfly_graph()

        # Test the default constructor.
        solver_default = mut.MaxCliqueSolverViaMip()
        self.assertIsNone(solver_default.GetInitialGuess())
        self.assertEqual(len(solver_default.GetSolverOptions().options), 0)

        # Test the argument constructor.
        solver_options = SolverOptions()
        solver_options.SetOption(CommonSolverOption.kPrintToConsole, True)
        initial_guess = np.ones(graph.shape[0])
        solver = mut.MaxCliqueSolverViaMip(
            solver_options=solver_options, initial_guess=initial_guess
        )
        # Test the getters.
        numpy_compare.assert_equal(solver.GetInitialGuess(), initial_guess)
        self.assertTrue(
            solver.GetSolverOptions().options["Drake"]["kPrintToConsole"],
        )

        # Test the setters.
        new_guess = np.zeros(graph.shape[0])
        solver.SetInitialGuess(initial_guess=new_guess)
        numpy_compare.assert_equal(solver.GetInitialGuess(), new_guess)

        new_options = SolverOptions()
        solver.SetSolverOptions(solver_options=new_options)
        self.assertEqual(len(solver.GetSolverOptions().options), 0)

        # Test solve max clique.
        if GurobiOrMosekSolverAvailable():
            max_clique = solver.SolveMaxClique(graph)
            # Butteryfly graph has a max clique of 3.
            self.assertEqual(max_clique.sum(), 3)

    def test_max_clique_solver_via_greedy_methods(self):
        graph = self._butteryfly_graph()

        # Test the default constructor.
        solver = mut.MaxCliqueSolverViaGreedy()

        # Test solve max clique.
        max_clique = solver.SolveMaxClique(graph)
        # Butteryfly graph has a max clique of 3.
        self.assertEqual(max_clique.sum(), 3)

    def test_min_clique_cover_solver_base_subclassable(self):
        class DummyMinCliqueCoverSolver(mut.MinCliqueCoverSolverBase):
            def __init__(self, name):
                mut.MinCliqueCoverSolverBase.__init__(self)
                self.name = name

            def DoSolveMinCliqueCover(self, adjacency_matrix, partition):
                # returns all points are in a clique
                return [set(i for i in range(adjacency_matrix.shape[0]))]

        name = "dummy"
        solver = DummyMinCliqueCoverSolver(name=name)
        graph = self._butteryfly_graph()
        self.assertEqual(name, solver.name)
        self.assertEqual(
            solver.SolveMinCliqueCover(adjacency_matrix=graph),
            [set(i for i in range(5))],
        )

    def test_min_clique_cover_solver_via_greedy_methods(self):
        graph = self._butteryfly_graph()

        # Test the default constructor.
        max_clique_solver = mut.MaxCliqueSolverViaGreedy()
        solver = mut.MinCliqueCoverSolverViaGreedy(
            max_clique_solver=max_clique_solver, min_clique_size=3
        )

        # Test solve min clique cover.
        min_clique_cover = solver.SolveMinCliqueCover(
            adjacency_matrix=graph, partition=False
        )
        self.assertEqual(len(min_clique_cover), 2)

        self.assertEqual(solver.get_min_clique_size(), 3)
        solver.set_min_clique_size(min_clique_size=1)
        self.assertEqual(solver.get_min_clique_size(), 1)
