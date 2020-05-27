import unittest

from pydrake.symbolic import Variable, Expression
from pydrake.systems.primitives import (
    ConstantVectorSource,
    SymbolicVectorSystem,
    SymbolicVectorSystem_,
)
from pydrake.systems.framework import EventStatus
from pydrake.systems.analysis import (
    RungeKutta2Integrator_,
    RegionOfAttraction,
    RegionOfAttractionOptions,
    Simulator,
    SimulatorStatus,
)
from pydrake.trajectories import PiecewisePolynomial


class TestAnalysis(unittest.TestCase):
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

    def test_simulator_status(self):
        SimulatorStatus.ReturnReason.kReachedBoundaryTime
        SimulatorStatus.ReturnReason.kReachedTerminationCondition
        SimulatorStatus.ReturnReason.kEventHandlerFailed

        system = ConstantVectorSource([1.])
        simulator = Simulator(system)
        status = simulator.AdvanceTo(1.)
        self.assertEqual(
            status.FormatMessage(),
            "Simulator successfully reached the boundary time (1.0).")
        self.assertTrue(status.succeeded())
        self.assertEqual(status.boundary_time(), 1.)
        self.assertEqual(status.return_time(), 1.)
        self.assertEqual(
            status.reason(),
            SimulatorStatus.ReturnReason.kReachedBoundaryTime)
        self.assertIsNone(status.system())
        self.assertEqual(status.message(), "")
        self.assertTrue(status.IsIdenticalStatus(other=status))

    def test_system_monitor(self):
        x = Variable("x")
        sys = SymbolicVectorSystem(state=[x], dynamics=[-x+x**3])
        simulator = Simulator(sys)

        def monitor(root_context):
            context = sys.GetMyContextFromRoot(root_context)
            if context.get_time() >= 1.:
                return EventStatus.ReachedTermination(sys, "Time reached")
            else:
                return EventStatus.DidNothing()

        self.assertIsNone(simulator.get_monitor())
        simulator.set_monitor(monitor)
        self.assertIsNotNone(simulator.get_monitor())
        status = simulator.AdvanceTo(2.)
        self.assertEqual(
            status.reason(),
            SimulatorStatus.ReturnReason.kReachedTerminationCondition)
        self.assertLess(status.return_time(), 1.1)
        simulator.clear_monitor()
        self.assertIsNone(simulator.get_monitor())
