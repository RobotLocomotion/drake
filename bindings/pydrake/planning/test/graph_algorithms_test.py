import numpy as np
import scipy.sparse as sp
import unittest

import pydrake.planning as mut
from pydrake.solvers import SolverOptions, CommonSolverOption
from pydrake.common.test_utilities import numpy_compare


class TestGraphAlgorithms(unittest.TestCase):
    """Tests the classes and methods defined in
     pydrake.planning.graph_algorithms.
    """
    def _butteryfly_graph(self):
        adjacency = np.array([
            [0, 1, 1, 0, 0],
            [1, 0, 1, 0, 0],
            [1, 1, 0, 1, 1],
            [0, 0, 1, 0, 1],
            [0, 0, 1, 1, 0],
        ]).astype(bool)
        assert np.all(adjacency == adjacency.T)
        return sp.csc_matrix(adjacency, shape=(5,5))

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
            np.ones(graph.shape[0])
        )

    def test_max_clique_via_mip_methods(self):
        graph = self._butteryfly_graph()

        # Test the default constructor.
        solver_default = mut.MaxCliqueSolverViaMip()
        self.assertIsNone(solver_default.get_initial_guess())
        self.assertFalse(
            solver_default.get_solver_options().get_print_to_console())

        # Test the argument constructor.
        solver_options = SolverOptions()
        solver_options.SetOption(CommonSolverOption.kPrintToConsole, True)
        initial_guess = np.ones(graph.shape[0])
        solver = mut.MaxCliqueSolverViaMip(solver_options=solver_options,
                                           initial_guess=initial_guess)
        # Test the getters
        numpy_compare.assert_equal(
            solver.get_initial_guess(), initial_guess
        )
        self.assertTrue(solver.get_solver_options().get_print_to_console())

        # Test the setters
        new_guess = np.zeros(graph.shape[0])
        solver.set_initial_guess(initial_guess=new_guess)
        numpy_compare.assert_equal(
            solver.get_initial_guess(), new_guess
        )

        new_options = SolverOptions()
        solver.set_solver_options(solver_options=new_options)
        self.assertFalse(solver.get_solver_options().get_print_to_console())

        # Test solve max clique
        max_clique = solver.SolveMaxClique(graph)
        # Butteryfly graph has a max clique of 3.
        self.assertEqual(max_clique.sum(), 3)
