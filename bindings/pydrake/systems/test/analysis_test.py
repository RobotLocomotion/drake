import copy
import unittest

from pydrake.common.test_utilities import numpy_compare
from pydrake.symbolic import Variable, Expression
from pydrake.autodiffutils import AutoDiffXd
from pydrake.systems.primitives import (
    ConstantVectorSource,
    ConstantVectorSource_,
    SymbolicVectorSystem,
    SymbolicVectorSystem_,
)
from pydrake.systems.framework import Context_, EventStatus
from pydrake.systems.analysis import (
    ApplySimulatorConfig,
    ExtractSimulatorConfig,
    InitializeParams,
    PrintSimulatorStatistics,
    ResetIntegratorFromFlags,
    RungeKutta2Integrator_,
    RungeKutta2Integrator,
    RungeKutta3Integrator,
    RungeKutta3Integrator_,
    RegionOfAttraction,
    RegionOfAttractionOptions,
    Simulator,
    Simulator_,
    SimulatorConfig,
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
        numpy_compare.assert_equal(options.state_variables, [x])
        options.use_implicit_dynamics = False
        V = RegionOfAttraction(system=sys, context=context, options=options)
        self.assertIsInstance(V, Expression)
        self.assertEqual(repr(options), "".join([
            "RegionOfAttractionOptions(",
            "lyapunov_candidate=pow(x, 2), ",
            "state_variables=[Variable('x', Continuous)], "
            "use_implicit_dynamics=False)"]))

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

    @numpy_compare.check_nonsymbolic_types
    def test_simulator_api(self, T):
        """Tests basic Simulator API."""
        # TODO(eric.cousineau): Migrate tests from `general_test.py` to here.
        system = ConstantVectorSource_[T]([1.])
        simulator = Simulator_[T](system=system)
        simulator = Simulator_[T](
            system=system, context=system.CreateDefaultContext())

        simulator.Initialize()
        initialize_params = InitializeParams(
            suppress_initialization_events=True)
        self.assertEqual(
            repr(initialize_params),
            "InitializeParams(suppress_initialization_events=True)")
        copy.copy(initialize_params)
        simulator.Initialize(params=initialize_params)

        simulator.AdvanceTo(boundary_time=0.0)
        simulator.AdvancePendingEvents()

        monitor_called_count = 0

        def monitor(root_context):
            nonlocal monitor_called_count
            monitor_called_count += 1
            return EventStatus.DidNothing()

        simulator.set_monitor(monitor=monitor)
        # N.B. This will be round-trip wrapped via pybind11, but should be the
        # same function underneath.w
        monitor_from_pybind = simulator.get_monitor()
        self.assertIsNot(monitor_from_pybind, monitor)
        self.assertEqual(monitor_called_count, 0)
        monitor_from_pybind(simulator.get_context())
        self.assertEqual(monitor_called_count, 1)
        simulator.clear_monitor()

        self.assertIsInstance(simulator.get_context(), Context_[T])
        self.assertIs(simulator.get_context(), simulator.get_mutable_context())
        self.assertTrue(simulator.has_context())

        self.assertIsInstance(
            simulator.get_integrator(), RungeKutta3Integrator_[T])
        self.assertIs(
            simulator.get_integrator(), simulator.get_mutable_integrator())
        simulator.reset_context(context=simulator.get_context().Clone())

        simulator.set_publish_every_time_step(publish=True)
        simulator.set_publish_at_initialization(publish=True)
        simulator.set_target_realtime_rate(realtime_rate=0.0)
        self.assertEqual(simulator.get_target_realtime_rate(), 0.0)
        self.assertIsInstance(simulator.get_actual_realtime_rate(), float)
        simulator.ResetStatistics()

        self.assertEqual(simulator.get_num_publishes(), 0)
        self.assertEqual(simulator.get_num_steps_taken(), 0)
        self.assertEqual(simulator.get_num_discrete_updates(), 0)
        self.assertEqual(simulator.get_num_unrestricted_updates(), 0)

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
            new_integrator = ResetIntegratorFromFlags(
                simulator=simulator, scheme="runge_kutta2",
                max_step_size=0.001)
            self.assertIsInstance(new_integrator, RungeKutta2Integrator_[T])

    def test_simulator_config(self):
        SimulatorConfig()
        config = SimulatorConfig(target_realtime_rate=2.0)
        self.assertEqual(config.target_realtime_rate, 2.0)
        self.assertIn("target_realtime_rate", repr(config))
        copy.copy(config)

    def test_simulator_config_functions(self):
        for T in (float, AutoDiffXd):
            source = ConstantVectorSource_[T]([2, 3])
            simulator = Simulator_[T](source)
            config = ExtractSimulatorConfig(simulator)
            config.target_realtime_rate = 100.0
            ApplySimulatorConfig(config=config, simulator=simulator)
            self.assertEqual(simulator.get_target_realtime_rate(), 100.0)

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
