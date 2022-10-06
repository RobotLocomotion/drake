import unittest

from pydrake.multibody.benchmarks import (MassDamperSpringAnalyticalSolution)


class TestBenchmarks(unittest.TestCase):

    def test_mass_damper_spring_analytical_solution(self):
        sol = MassDamperSpringAnalyticalSolution(mass=1, b=1, k=1)
        sol.SetInitialValue(x0=1, xDt0=0)
        self.assertEqual(sol.get_x(0), 1)
        self.assertEqual(sol.get_xDt(0), 0)
        self.assertEqual(sol.get_xDtDt(0), -1)
