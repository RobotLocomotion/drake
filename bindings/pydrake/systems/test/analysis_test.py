import copy
import unittest

from pydrake.common.test_utilities import numpy_compare
from pydrake.math import isnan
from pydrake.symbolic import Variable, Expression
from pydrake.autodiffutils import AutoDiffXd
from pydrake.systems.primitives import (
    ConstantVectorSource,
    ConstantVectorSource_,
    FirstOrderLowPassFilter_,
    SymbolicVectorSystem,
    SymbolicVectorSystem_,
)
from pydrake.systems.framework import Context_, EventStatus
from pydrake.systems.analysis import (
    ApplySimulatorConfig,
    ExtractSimulatorConfig,
    InitializeParams,
    IntegratorBase_,
    PrintSimulatorStatistics,
    ResetIntegratorFromFlags,
    RungeKutta2Integrator, RungeKutta2Integrator_,
    RungeKutta3Integrator, RungeKutta3Integrator_,
    RegionOfAttraction,
    RegionOfAttractionOptions,
    Simulator,
    Simulator_,
    SimulatorConfig,
    SimulatorStatus,
)
from pydrake.trajectories import PiecewisePolynomial, PiecewisePolynomial_


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
        options.solver_id = None
        options.solver_options = None
        V = RegionOfAttraction(system=sys, context=context, options=options)
        self.assertIsInstance(V, Expression)
        self.assertGreater(len(repr(options)), 0)
        self.assertIn("use_implicit_dynamics", repr(options))

    def test_integrator_constructors(self):
        """Test all constructors for all integrator types."""
        system = ConstantVectorSource([1])
        context = system.CreateDefaultContext()
        RungeKutta2Integrator(system=system, max_step_size=0.01)
        RungeKutta2Integrator(
            system=system, max_step_size=0.01, context=context)
        RungeKutta3Integrator(system=system)
        RungeKutta3Integrator(system=system, context=context)

    @numpy_compare.check_nonsymbolic_types
    def test_integrator_api(self, T):
        system = FirstOrderLowPassFilter_[T](time_constant=1.0, size=1)
        context = system.CreateDefaultContext()
        system.get_input_port().FixValue(context, [1.0])

        integrator = RungeKutta3Integrator_[T](system=system)
        self.assertIsInstance(integrator, IntegratorBase_[T])

        # WARNING: IntegratorBase.get_context() could segfault if context is
        # not set.
        integrator.reset_context(context=context)
        self.assertIs(integrator.get_context(), context)
        self.assertIs(integrator.get_mutable_context(), context)

        target_accuracy = 1E-6
        integrator.set_target_accuracy(accuracy=target_accuracy)
        self.assertEqual(integrator.get_target_accuracy(), target_accuracy)

        maximum_step_size = 0.2
        integrator.set_maximum_step_size(max_step_size=maximum_step_size)
        self.assertEqual(integrator.get_maximum_step_size(), maximum_step_size)

        minimum_step_size = 2E-2
        integrator.set_requested_minimum_step_size(
            min_step_size=minimum_step_size)
        self.assertEqual(
            integrator.get_requested_minimum_step_size(), minimum_step_size)

        integrator.set_throw_on_minimum_step_size_violation(throws=True)
        self.assertTrue(integrator.get_throw_on_minimum_step_size_violation())

        integrator.set_fixed_step_mode(flag=True)
        self.assertTrue(integrator.get_fixed_step_mode())

        integrator.Initialize()

        integrator.StartDenseIntegration()
        dense_output = integrator.get_dense_output()
        self.assertIsInstance(dense_output, PiecewisePolynomial_[T])
        self.assertIs(integrator.StopDenseIntegration(), dense_output)

        self.assertEqual(integrator.get_num_substep_failures(), 0)
        self.assertEqual(
            integrator.get_num_step_shrinkages_from_substep_failures(), 0)
        self.assertEqual(
            integrator.get_num_step_shrinkages_from_error_control(), 0)
        self.assertEqual(integrator.get_num_derivative_evaluations(), 0)
        self.assertTrue(isnan(integrator.get_actual_initial_step_size_taken()))
        self.assertTrue(
            isnan(integrator.get_smallest_adapted_step_size_taken()))
        self.assertTrue(isnan(integrator.get_largest_step_size_taken()))
        self.assertEqual(integrator.get_num_steps_taken(), 0)
        integrator.ResetStatistics()

        integrator.Reset()

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

        simulator.AdvanceTo(boundary_time=0.0, interruptible=False)
        simulator.AdvancePendingEvents()
        simulator.AdvanceTo(boundary_time=0.1, interruptible=True)

        monitor_called_count = 0

        def monitor(root_context):
            nonlocal monitor_called_count
            monitor_called_count += 1
            return EventStatus.DidNothing()

        simulator.set_monitor(monitor=monitor)
        # N.B. This will be round-trip wrapped via pybind11, but should be the
        # same function underneath.
        monitor_from_pybind = simulator.get_monitor()
        self.assertIsNot(monitor_from_pybind, monitor)
        self.assertEqual(monitor_called_count, 0)
        monitor_from_pybind(simulator.get_context())
        self.assertEqual(monitor_called_count, 1)
        simulator.clear_monitor()

        monitor_no_return_called_count = 0

        def monitor_no_return(root_context):
            nonlocal monitor_no_return_called_count
            monitor_no_return_called_count += 1

        simulator.set_monitor(monitor_no_return)
        monitor_from_pybind = simulator.get_monitor()
        status = monitor_from_pybind(simulator.get_context())
        self.assertEqual(status.severity(), EventStatus.Severity.kDidNothing)
        self.assertEqual(monitor_no_return_called_count, 1)
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
            # N.B. We suppress returning anything to test the binding's ability
            # to handle `None` return type.

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
