import unittest

from pydrake.symbolic import Variable, Expression
from pydrake.systems.primitives import (
    SymbolicVectorSystem,
    SymbolicVectorSystem_
)
from pydrake.systems.analysis import (
    RungeKutta2Integrator_,
    RegionOfAttraction,
    RegionOfAttractionOptions,
    Simulator
)
from pydrake.trajectories import PiecewisePolynomial


class AnalysisTest(unittest.TestCase):
    def test_region_of_attraction(self):
        x = Variable("x")
        sys = SymbolicVectorSystem(state=[x], dynamics=[-x+x**3])
        context = sys.CreateDefaultContext()
        options = RegionOfAttractionOptions()
        options.lyapunov_candidate = x*x
        options.state_variables = [x]
        V = RegionOfAttraction(system=sys, context=context, options=options)

    def test_symbolic_integrators(self):
        x = Variable("x")
        sys = SymbolicVectorSystem_[Expression](state=[x], dynamics=[-x+x**3])
        context = sys.CreateDefaultContext()

        max_h = 0.1
        RungeKutta2Integrator_[Expression](sys, max_h, context)

    def test_dense_integration(self):
        x = Variable("x")
        sys = SymbolicVectorSystem(state=[x], dynamics=[-x+x**3])
        simulator = Simulator(sys)
        integrator = simulator.get_mutable_integrator()
        self.assertIsNone(integrator.get_dense_output())
        integrator.StartDenseIntegration()
        pp = integrator.get_dense_output()
        self.assertIsInstance(pp, PiecewisePolynomial)
        simulator.AdvanceTo(1.0)
        self.assertIs(pp, integrator.StopDenseIntegration())
        self.assertEqual(pp.start_time(), 0.0)
        self.assertEqual(pp.end_time(), 1.0)
        self.assertIsNone(integrator.get_dense_output())
