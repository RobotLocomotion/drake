# -*- coding: utf-8 -*-

from __future__ import print_function

import pydrake.systems.framework as mut

import copy
import warnings

import unittest
import numpy as np

from pydrake.autodiffutils import (
    AutoDiffXd,
    )
from pydrake.examples.pendulum import PendulumPlant
from pydrake.symbolic import (
    Expression,
    )
from pydrake.systems.analysis import (
    IntegratorBase_,
    RungeKutta2Integrator, RungeKutta3Integrator,
    Simulator, Simulator_,
    )
from pydrake.systems.framework import (
    AbstractValue,
    BasicVector, BasicVector_,
    Context_,
    ContinuousState, ContinuousState_,
    Diagram, Diagram_,
    DiagramBuilder, DiagramBuilder_,
    DiscreteUpdateEvent_,
    DiscreteValues_,
    Event, Event_,
    InputPort_,
    kUseDefaultName,
    LeafContext_,
    LeafSystem_,
    OutputPort_,
    Parameters_,
    PublishEvent, PublishEvent_,
    State_,
    Subvector_,
    Supervector_,
    System_,
    SystemOutput_,
    VectorBase, VectorBase_,
    TriggerType,
    VectorSystem_,
    )
from pydrake.systems.primitives import (
    Adder, Adder_,
    AffineSystem,
    ConstantValueSource,
    ConstantVectorSource, ConstantVectorSource_,
    Integrator,
    LinearSystem,
    SignalLogger,
    )

# TODO(eric.cousineau): The scope of this test file and and `custom_test.py`
# is poor. Move these tests into `framework_test` and `analysis_test`, and
# ensure that the tests reflect this, even if there is some coupling.


class TestGeneral(unittest.TestCase):
    def _check_instantiations(self, template, supports_symbolic=True):
        default_cls = template[None]
        self.assertTrue(template[float] is default_cls)
        self.assertTrue(template[AutoDiffXd] is not default_cls)
        if supports_symbolic:
            self.assertTrue(template[Expression] is not default_cls)

    def _compare_system_instances(self, lhs, rhs):
        # Compares two different scalar type instantiation instances of a
        # system.
        self.assertEqual(lhs.get_num_input_ports(), rhs.get_num_input_ports())
        self.assertEqual(
            lhs.get_num_output_ports(), rhs.get_num_output_ports())
        for i in range(lhs.get_num_input_ports()):
            lhs_port = lhs.get_input_port(i)
            rhs_port = rhs.get_input_port(i)
            self.assertEqual(lhs_port.size(), rhs_port.size())
        for i in range(lhs.get_num_output_ports()):
            lhs_port = lhs.get_output_port(i)
            rhs_port = rhs.get_output_port(i)
            self.assertEqual(lhs_port.size(), rhs_port.size())

    def test_system_base_api(self):
        # Test a system with a different number of inputs from outputs.
        system = Adder(3, 10)
        self.assertEqual(system.get_num_input_ports(), 3)
        self.assertEqual(system.get_num_output_ports(), 1)
        self.assertEqual(system.GetInputPort("u1").get_index(), 1)
        self.assertEqual(system.GetOutputPort("sum").get_index(), 0)
        # TODO(eric.cousineau): Consolidate the main API tests for `System`
        # to this test point.

    def test_context_api(self):
        system = Adder(3, 10)
        context = system.CreateDefaultContext()
        self.assertIsInstance(
            context.get_continuous_state(), ContinuousState)
        self.assertIsInstance(
            context.get_mutable_continuous_state(), ContinuousState)
        self.assertIsInstance(
            context.get_continuous_state_vector(), VectorBase)
        self.assertIsInstance(
            context.get_mutable_continuous_state_vector(), VectorBase)
        # TODO(eric.cousineau): Consolidate main API tests for `Context` here.

        pendulum = PendulumPlant()
        context = pendulum.CreateDefaultContext()
        self.assertEqual(context.num_numeric_parameter_groups(), 1)
        self.assertTrue(
            context.get_parameters().get_numeric_parameter(0) is
            context.get_numeric_parameter(index=0))
        self.assertEqual(context.num_abstract_parameters(), 0)
        # TODO(russt): Bind _Declare*Parameter or find an example with an
        # abstract parameter to actually call this method.
        self.assertTrue(hasattr(context, "get_abstract_parameter"))

    def test_event_api(self):
        # TriggerType - existence check.
        TriggerType.kUnknown
        TriggerType.kInitialization
        TriggerType.kForced
        TriggerType.kTimed
        TriggerType.kPeriodic
        TriggerType.kPerStep
        TriggerType.kWitness

        # PublishEvent.
        # TODO(eric.cousineau): Test other event types when it is useful to
        # expose them.

        def callback(context, event): pass

        event = PublishEvent(
            trigger_type=TriggerType.kInitialization, callback=callback)
        self.assertIsInstance(event, Event)
        self.assertEqual(event.get_trigger_type(), TriggerType.kInitialization)

    def test_instantiations(self):
        # Quick check of instantions for given types.
        # N.B. These checks are ordered according to their binding definitions
        # in the corresponding source file.
        # `analysis_py.cc`
        self._check_instantiations(IntegratorBase_, False)
        self._check_instantiations(Simulator_, False)
        # `analysis_py_semantics.cc`
        self._check_instantiations(Context_)
        self._check_instantiations(LeafContext_)
        self._check_instantiations(Event_)
        self._check_instantiations(PublishEvent_)
        self._check_instantiations(DiscreteUpdateEvent_)
        self._check_instantiations(DiagramBuilder_)
        self._check_instantiations(OutputPort_)
        self._check_instantiations(SystemOutput_)
        self._check_instantiations(InputPort_)
        self._check_instantiations(Parameters_)
        self._check_instantiations(State_)
        self._check_instantiations(ContinuousState_)
        self._check_instantiations(DiscreteValues_)
        # `analysis_py_systems.cc`
        self._check_instantiations(System_)
        self._check_instantiations(LeafSystem_)
        self._check_instantiations(Diagram_)
        self._check_instantiations(VectorSystem_)
        # `analysis_py_values.cc`
        self._check_instantiations(VectorBase_)
        self._check_instantiations(BasicVector_)
        self._check_instantiations(Supervector_)
        self._check_instantiations(Subvector_)

    def test_scalar_type_conversion(self):
        for T in [float, AutoDiffXd, Expression]:
            system = Adder_[T](1, 1)
            # N.B. Current scalar conversion does not permit conversion to and
            # from the same type.
            if T != AutoDiffXd:
                methods = [Adder_[T].ToAutoDiffXd, Adder_[T].ToAutoDiffXdMaybe]
                for method in methods:
                    system_ad = method(system)
                    self.assertIsInstance(system_ad, System_[AutoDiffXd])
                    self._compare_system_instances(system, system_ad)
            if T != Expression:
                methods = [Adder_[T].ToSymbolic, Adder_[T].ToSymbolicMaybe]
                for method in methods:
                    system_sym = method(system)
                    self.assertIsInstance(system_sym, System_[Expression])
                    self._compare_system_instances(system, system_sym)

    def test_simulator_ctor(self):
        # Tests a simple simulation for supported scalar types.
        for T in [float, AutoDiffXd]:
            # Create simple system.
            system = ConstantVectorSource_[T]([1.])

            def check_output(context):
                # Check number of output ports and value for a given context.
                output = system.AllocateOutput()
                self.assertEqual(output.get_num_ports(), 1)
                system.CalcOutput(context, output)
                if T == float:
                    value = output.get_vector_data(0).get_value()
                    self.assertTrue(np.allclose([1], value))
                elif T == AutoDiffXd:
                    value = output.get_vector_data(0)._get_value_copy()
                    # TODO(eric.cousineau): Define `isfinite` ufunc, if
                    # possible, to use for `np.allclose`.
                    self.assertEqual(value.shape, (1,))
                    self.assertEqual(value[0], AutoDiffXd(1.))
                else:
                    raise RuntimeError("Bad T: {}".format(T))

            # Create simulator with basic constructor.
            simulator = Simulator_[T](system)
            simulator.Initialize()
            simulator.set_target_realtime_rate(0)
            simulator.set_publish_every_time_step(True)
            self.assertTrue(simulator.get_context() is
                            simulator.get_mutable_context())
            check_output(simulator.get_context())
            simulator.StepTo(1)

            # Create simulator specifying context.
            context = system.CreateDefaultContext()
            context.set_time(0.)

            context.set_accuracy(1e-4)
            self.assertEqual(context.get_accuracy(), 1e-4)

            # @note `simulator` now owns `context`.
            simulator = Simulator_[T](system, context)
            self.assertTrue(simulator.get_context() is context)
            check_output(context)
            simulator.StepTo(1)

    def test_copy(self):
        # Copy a context using `deepcopy` or `clone`.
        system = ConstantVectorSource([1])
        context = system.CreateDefaultContext()
        context_copies = [
            copy.copy(context),
            copy.deepcopy(context),
            context.Clone(),
        ]
        # TODO(eric.cousineau): Compare copies.
        for context_copy in context_copies:
            self.assertTrue(context_copy is not context)

    def test_diagram_simulation(self):
        # Similar to: //systems/framework:diagram_test, ExampleDiagram
        size = 3

        builder = DiagramBuilder()
        adder0 = builder.AddSystem(Adder(2, size))
        adder0.set_name("adder0")
        adder1 = builder.AddSystem(Adder(2, size))
        adder1.set_name("adder1")

        integrator = builder.AddSystem(Integrator(size))
        integrator.set_name("integrator")

        builder.Connect(adder0.get_output_port(0), adder1.get_input_port(0))
        builder.Connect(adder1.get_output_port(0),
                        integrator.get_input_port(0))

        # Exercise naming variants.
        builder.ExportInput(adder0.get_input_port(0))
        builder.ExportInput(adder0.get_input_port(1), kUseDefaultName)
        builder.ExportInput(adder1.get_input_port(1), "third_input")
        builder.ExportOutput(integrator.get_output_port(0), "result")

        diagram = builder.Build()
        # TODO(eric.cousineau): Figure out unicode handling if needed.
        # See //systems/framework/test/diagram_test.cc:349 (sha: bc84e73)
        # for an example name.
        diagram.set_name("test_diagram")

        simulator = Simulator(diagram)
        context = simulator.get_mutable_context()

        # Create and attach inputs.
        # TODO(eric.cousineau): Not seeing any assertions being printed if no
        # inputs are connected. Need to check this behavior.
        input0 = np.array([0.1, 0.2, 0.3])
        context.FixInputPort(0, input0)
        input1 = np.array([0.02, 0.03, 0.04])
        context.FixInputPort(1, input1)
        input2 = BasicVector([0.003, 0.004, 0.005])
        context.FixInputPort(2, input2)  # Test the BasicVector overload.

        # Test __str__ methods.
        self.assertRegexpMatches(str(context), "integrator")
        self.assertEqual(str(input2), "[0.003, 0.004, 0.005]")

        # Initialize integrator states.
        integrator_xc = (
            diagram.GetMutableSubsystemState(integrator, context)
                   .get_mutable_continuous_state().get_vector())
        integrator_xc.SetFromVector([0, 1, 2])

        simulator.Initialize()

        # Simulate briefly, and take full-context snapshots at intermediate
        # points.
        n = 6
        times = np.linspace(0, 1, n)
        context_log = []
        for t in times:
            simulator.StepTo(t)
            # Record snapshot of *entire* context.
            context_log.append(context.Clone())

        xc_initial = np.array([0, 1, 2])
        xc_final = np.array([0.123, 1.234, 2.345])

        for i, context_i in enumerate(context_log):
            t = times[i]
            self.assertEqual(context_i.get_time(), t)
            xc = context_i.get_continuous_state_vector().CopyToVector()
            xc_expected = (float(i) / (n - 1) * (xc_final - xc_initial) +
                           xc_initial)
            print("xc[t = {}] = {}".format(t, xc))
            self.assertTrue(np.allclose(xc, xc_expected))

    def test_simulator_integrator_manipulation(self):
        system = ConstantVectorSource([1])

        # Create simulator with basic constructor.
        simulator = Simulator(system)
        simulator.Initialize()
        simulator.set_target_realtime_rate(0)

        integrator = simulator.get_mutable_integrator()

        target_accuracy = 1E-6
        integrator.set_target_accuracy(target_accuracy)
        self.assertEqual(integrator.get_target_accuracy(), target_accuracy)

        maximum_step_size = 0.2
        integrator.set_maximum_step_size(maximum_step_size)
        self.assertEqual(integrator.get_maximum_step_size(), maximum_step_size)

        minimum_step_size = 2E-2
        integrator.set_requested_minimum_step_size(minimum_step_size)
        self.assertEqual(integrator.get_requested_minimum_step_size(),
                         minimum_step_size)

        integrator.set_throw_on_minimum_step_size_violation(True)
        self.assertTrue(integrator.get_throw_on_minimum_step_size_violation())

        integrator.set_fixed_step_mode(True)
        self.assertTrue(integrator.get_fixed_step_mode())

        const_integrator = simulator.get_integrator()
        self.assertTrue(const_integrator is integrator)

        # Test context-less constructors for
        # integrator types.
        test_integrator = RungeKutta2Integrator(
            system=system, max_step_size=0.01)
        test_integrator = RungeKutta3Integrator(system=system)

        # Test simulator's reset_integrator,
        # and also the full constructors for
        # all integrator types.
        simulator.reset_integrator(
            RungeKutta2Integrator(
                system=system,
                max_step_size=0.01,
                context=simulator.get_mutable_context()))

        simulator.reset_integrator(
            RungeKutta3Integrator(
                system=system,
                context=simulator.get_mutable_context()))

    def test_eval(self):
        """Tests evaluation (e.g. caching, API sugars, etc.)."""
        # `Eval` and `EvalAbstract`: Test with constant systems.
        model_values = [
            AbstractValue.Make("Hello World"),
            AbstractValue.Make(BasicVector([1., 2., 3.])),
        ]
        for model_value in model_values:
            is_abstract = not isinstance(model_value.get_value(), BasicVector)
            if is_abstract:
                zoh = ConstantValueSource(copy.copy(model_value))
            else:
                zoh = ConstantVectorSource(model_value.get_value().get_value())
            context = zoh.CreateDefaultContext()
            output_port = zoh.get_output_port(0)
            value_abstract = output_port.EvalAbstract(context)
            value = output_port.Eval(context)
            self.assertEqual(type(value_abstract), type(model_value))
            self.assertEqual(type(value), type(model_value.get_value()))
            if is_abstract:
                check = self.assertEqual
            else:

                def check(a, b):
                    self.assertEqual(type(a.get_value()), type(b.get_value()))
                    np.testing.assert_equal(a.get_value(), b.get_value())

            check(value_abstract.get_value(), model_value.get_value())
            check(value, model_value.get_value())
