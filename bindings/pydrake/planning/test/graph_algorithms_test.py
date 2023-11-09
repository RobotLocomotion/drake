import numpy as np
import scipy.sparse as sp
import unittest

import pydrake.planning as mut
from pydrake.solvers import SolverOptions, CommonSolverOption
from pydrake.common.test_utilities import numpy_compare


class TestGraphAlgorithms(unittest.TestCase):
    """Tests the classes and methods defined in pydrake.planning.graph_algorithms.
    """
    def _butteryfly_graph(self):
        adjacency = np.array([
            [0,1,1,0,0],
            [1,0,1,0,0],
            [1,1,0,1,1],
            [0,0,1,0,1],
            [0,0,1,1,0],
        ])
        assert np.all(adjacency == adjacency.T)
        return sp.csr_matrix(adjacency)

    def test_max_clique_via_mip_methods(self):
        graph = self._butteryfly_graph()

        solver1 = mut.MaxCliqueSolverViaMip()
        self.assertIsNone(solver1.get_initial_guess())
        self.assertEqual(solver1.get_solver_options(), SolverOptions())

        solver_options = SolverOptions()
        solver_options.SetOption(CommonSolverOption.kPrintToConsole, 1)
        initial_guess = np.ones(graph.shape[0])
        solver2 = mut.MaxCliqueSolverViaMip(solver_options=solver_options,
                                            initial_guess=initial_guess)
        numpy_compare.assert_equal(
            solver2.get_initial_guess(), initial_guess
        )
        self.assertEqual(solver2.get_solver_options(), solver_options)

        new_guess = np.zeros(graph.shape[0])
        solver2.set_initial_guess(initial_guess=new_guess)
        numpy_compare.assert_equal(
            solver2.get_initial_guess(), new_guess
        )

    def test_max_clique_options(self):
        options_default = mut.MaxCliqueOptions()
        self.assertIsInstance(options_default.solver, mut.MaxCliqueSolverViaMip)

        options_arg = mut.MaxCliqueOptions(m_solver = mut.MaxCliqueSolverViaMip())
        self.assertIsInstance(options_arg.solver, mut.MaxCliqueSolverViaMip)

    def test_calc_max_clique(self):
        graph = self._butteryfly_graph()
        options = mut.MaxCliqueOptions()
        max_clique = mut.CalcMaxClique(
            adjacency_matrix = graph,
            options = options
        )
        self.assertEqual(max_clique.sum(), 3)



