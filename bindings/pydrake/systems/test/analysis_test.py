import unittest

from pydrake.symbolic import Variable, Expression
from pydrake.autodiffutils import AutoDiffXd
from pydrake.systems.primitives import (
    ConstantVectorSource,
    ConstantVectorSource_,
    SymbolicVectorSystem,
    SymbolicVectorSystem_,
)
from pydrake.systems.framework import EventStatus
from pydrake.systems.analysis import (
    PrintSimulatorStatistics,
    ResetIntegratorFromFlags,
    RungeKutta2Integrator_,
    RungeKutta2Integrator, RungeKutta3Integrator,
    RegionOfAttraction,
    RegionOfAttractionOptions,
    Simulator,
    Simulator_,
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
        self.assertEqual(repr(options), "".join([
            "RegionOfAttractionOptions(",
            "lyapunov_candidate=pow(x, 2), ",
            "state_variables=[Variable('x', Continuous)])"]))

    def test_integrator_constructors(self):
        """Test all constructors for all integrator types."""
        sys = ConstantVectorSource([1])
        con = sys.CreateDefaultContext()
        RungeKutta2Integrator(system=sys, max_step_size=0.01)
        RungeKutta2Integrator(system=sys, max_step_size=0.01, context=con)
        RungeKutta3Integrator(system=sys)
        RungeKutta3Integrator(system=sys, context=con)

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

    def test_simulator_api(self):
        """Tests basic Simulator API."""
        # TODO(eric.cousineau): Migrate tests from `general_test.py` to here.
        system = ConstantVectorSource([1.])
        simulator = Simulator(system)
        self.assertIs(simulator.get_system(), system)

    def test_simulator_status(self):
        SimulatorStatus.ReturnReason.kReachedBoundaryTime
        SimulatorStatus.ReturnReason.kReachedTerminationCondition
        SimulatorStatus.ReturnReason.kEventHandlerFailed

        system = ConstantVectorSource([1.])
        simulator = Simulator(system)
        status = simulator.AdvanceTo(1.)
        self.assertRegex(
            status.FormatMessage(),
            "^Simulator successfully reached the boundary time")
        self.assertTrue(status.succeeded())
        self.assertEqual(status.boundary_time(), 1.)
        self.assertEqual(status.return_time(), 1.)
        self.assertEqual(
            status.reason(),
            SimulatorStatus.ReturnReason.kReachedBoundaryTime)
        self.assertIsNone(status.system())
        self.assertEqual(status.message(), "")
        self.assertTrue(status.IsIdenticalStatus(other=status))
        PrintSimulatorStatistics(simulator)

    def test_reset_integrator_from_flags(self):
        for T in (float, AutoDiffXd):
            source = ConstantVectorSource_[T]([2, 3])
            simulator = Simulator_[T](source)
            result = ResetIntegratorFromFlags(
                simulator=simulator, scheme="runge_kutta2",
                max_step_size=0.001)

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
