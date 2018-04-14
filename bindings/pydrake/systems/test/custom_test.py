# -*- coding: utf-8 -*-

from __future__ import print_function

import copy
import unittest
import warnings
import numpy as np

from pydrake.common.deprecation import DrakeDeprecationWarning
from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
from pydrake.systems.analysis import (
    Simulator, Simulator_,
    )
from pydrake.systems.framework import (
    AbstractValue,
    BasicVector, BasicVector_,
    Context, Context_,
    DiagramBuilder, DiagramBuilder_,
    kUseDefaultName,
    LeafSystem, LeafSystem_,
    PortDataType,
    PublishEvent, PublishEvent_,
    State,
    TriggerType,
    UnrestrictedUpdateEvent,
    VectorSystem, VectorSystem_,
    WitnessFunctionDirection,
    )
from pydrake.systems.scalar_conversion import TemplateSystem
from pydrake.systems.primitives import (
    ZeroOrderHold, ZeroOrderHold_,
    )

from pydrake.systems.test.test_util import (
    call_leaf_system_overrides,
    call_vector_system_overrides,
    )

from pydrake.common.test_utilities.deprecation import catch_drake_warnings


def noop(*args, **kwargs):
    # When a callback is required for an interface, but not useful for testing.
    pass


@TemplateSystem.define("CustomAdder_")
def CustomAdder_(T):

    class Impl(LeafSystem_[T]):
    # Reimplements `Adder`.
        def _construct(self, num_inputs, size, converter=None):
            LeafSystem_[T].__init__(self, converter=converter)
            self._num_inputs = num_inputs
            self._size = size
            for i in xrange(num_inputs):
                self.DeclareVectorInputPort(
                    "input{}".format(i), BasicVector_[T](size))
        self.DeclareVectorOutputPort(
            "sum", BasicVector_[T](size), self._calc_sum)

        def _construct_copy(self, other, converter=None):
            Impl._construct(
                self, other._num_inputs, other._size, converter=converter)

        def _calc_sum(self, context, sum_data):
            sum = sum_data.get_mutable_value()
            sum[:] = 0.
            for i in range(context.num_input_ports()):
                input_vector = self.EvalVectorInput(context, i)
                sum += input_vector.get_value()

    return Impl


@TemplateSystem.define("CustomVectorSystem_")
def CustomVectorSystem_(T):

    class Impl(VectorSystem_[T]):
        def _construct(self, is_discrete, converter=None):
            # VectorSystem only supports pure Continuous or pure Discrete.
            # Dimensions:
            #   1 Input, 2 States, 3 Outputs.
            VectorSystem_[T].__init__(self, 1, 3, converter=converter)
            self._is_discrete = is_discrete
            if self._is_discrete:
                self.DeclareDiscreteState(2)
            else:
                self.DeclareContinuousState(2)
            # Record calls for testing.
            self.has_called = []

        def _construct_copy(self, other, converter=None):
            Impl._construct(self, other._is_discrete, converter=converter)

        def DoCalcVectorOutput(self, context, u, x, y):
            y[:] = np.hstack([u, x])
            self.has_called.append("output")

        def DoCalcVectorTimeDerivatives(self, context, u, x, x_dot):
            x_dot[:] = x + u
            self.has_called.append("continuous")

        def DoCalcVectorDiscreteVariableUpdates(self, context, u, x, x_n):
            x_n[:] = x + 2*u
            self.has_called.append("discrete")

        def DoHasDirectFeedthrough(self, input_port, output_port):
            self.has_called.append("feedthrough")
            return True

            def _DoHasDirectFeedthrough(self, input_port, output_port):
                self.has_called.append("feedthrough")
                return True

    return Impl


# Default instantiations.
CustomAdder = CustomAdder_[None]
CustomVectorSystem = CustomVectorSystem_[None]


class TestCustom(unittest.TestCase):
    def assertArrayEqual(self, lhs, rhs):
        # TODO(eric.cousineau): Place in `pydrake.test.unittest_mixins`.
        lhs, rhs = np.array(lhs), np.array(rhs)
        if lhs.dtype == Expression or rhs.dtype == Expression:
            lhs, rhs = lhs.astype(Expression), rhs.astype(Expression)
            self.assertTrue(Expression.equal_to(lhs, rhs).all())
        else:
            self.assertTrue(np.allclose(lhs, rhs))

    def _create_adder_system(self, T):
        system = CustomAdder_[T](2, 3)
        return system

    def _fix_adder_inputs(self, context, T):
        self.assertEqual(context.num_input_ports(), 2)
        with catch_drake_warnings(expected_count=1):
            context.get_num_input_ports()
        context.FixInputPort(0, BasicVector_[T]([1, 2, 3]))
        context.FixInputPort(1, BasicVector_[T]([4, 5, 6]))

    def test_diagram_adder(self):
        system = CustomDiagram(2, 3)
        self.assertEqual(system.num_input_ports(), 2)
        with catch_drake_warnings(expected_count=1):
            system.get_num_input_ports()
        self.assertEqual(system.get_input_port(0).size(), 3)
        self.assertEqual(system.num_output_ports(), 1)
        with catch_drake_warnings(expected_count=1):
            system.get_num_output_ports()
        self.assertEqual(system.get_output_port(0).size(), 3)

    def test_adder_execution(self):
        for T in (float, AutoDiffXd, Expression):
            system = self._create_adder_system(T)
            context = system.CreateDefaultContext()
            self._fix_adder_inputs(context, T)
            output = system.AllocateOutput()
            self.assertEqual(output.num_ports(), 1)
            system.CalcOutput(context, output)
            value = output.get_vector_data(0).get_value()
            value_expected = np.array([5, 7, 9])
            self.assertArrayEqual(value_expected, value)

    def test_adder_simulation(self):
        for T in (float, AutoDiffXd):
            builder = DiagramBuilder_[T]()
            adder = builder.AddSystem(self._create_adder_system(T))
            adder.set_name("custom_adder")
            # Add ZOH so we can easily extract state.
            zoh = builder.AddSystem(ZeroOrderHold_[T](0.1, 3))
            zoh.set_name("zoh")

            builder.ExportInput(adder.get_input_port(0))
            builder.ExportInput(adder.get_input_port(1))
            builder.Connect(adder.get_output_port(0), zoh.get_input_port(0))
            diagram = builder.Build()
            context = diagram.CreateDefaultContext()
            self._fix_adder_inputs(context, T)

            simulator = Simulator_[T](diagram, context)
            simulator.Initialize()
            simulator.AdvanceTo(1)
            # Ensure that we have the outputs we want.
            value = (diagram.GetMutableSubsystemContext(zoh, context)
                     .get_discrete_state_vector().get_value())
            self.assertTrue(np.allclose([5, 7, 9], value))

    def test_leaf_system_overrides(self):
        map(self._check_leaf_system_overrides, (float, AutoDiffXd))

    def _check_leaf_system_overrides(self, T):
        test = self

        class TrivialSystem(LeafSystem_[T]):
            def __init__(self):
                LeafSystem_[T].__init__(self)
                self.called_publish = False
                self.called_feedthrough = False
                self.called_continuous = False
                self.called_discrete = False
                self.called_initialize = False
                self.called_periodic = False
                self.called_getwitness = False
                self.called_witness = False
                self.called_guard = False
                self.called_reset = False
                # Ensure we have desired overloads.
                self.DeclarePeriodicPublish(1.0)
                self.DeclarePeriodicPublish(1.0, 0)
                self.DeclarePeriodicPublish(period_sec=1.0, offset_sec=0.)
                self.DeclarePeriodicDiscreteUpdate(
                    period_sec=1.0, offset_sec=0.)
                self.DeclareInitializationEvent(
                    event=PublishEvent_[T](
                        trigger_type=TriggerType.kInitialization,
                        callback=self._on_initialize))
                self.DeclarePerStepEvent(
                    event=PublishEvent_[T](
                        trigger_type=TriggerType.kPerStep,
                        callback=self._on_per_step))
                self.DeclarePeriodicEvent(
                    period_sec=1.0,
                    offset_sec=0.0,
                    event=PublishEvent_[T](
                        trigger_type=TriggerType.kPeriodic,
                        callback=self._on_periodic))
                self.DeclareContinuousState(2)
                self.DeclareDiscreteState(1)
                # Ensure that we have inputs / outputs to call direct
                # feedthrough.
                self.DeclareInputPort(PortDataType.kVectorValued, 1)
                self.DeclareVectorInputPort(
                    name="test_input", model_vector=BasicVector_[T](1),
                    random_type=None)
                self.DeclareVectorOutputPort(BasicVector_[T](1), noop)
                self.witness = self.MakeWitnessFunction(
                    "witness", WitnessFunctionDirection.kCrossesZero,
                    self._witness)
                self.reset_witness = self.MakeWitnessFunction(
                    "reset", WitnessFunctionDirection.kCrossesZero,
                    self._guard, UnrestrictedUpdateEvent_[T](self._reset))

            def DoPublish(self, context, events):
                # Call base method to ensure we do not get recursion.
                LeafSystem.DoPublish(self, context, events)
                # N.B. We do not test for a singular call to `DoPublish`
                # (checking `assertFalse(self.called_publish)` first) because
                # the above `_DeclareInitializationEvent` will call both its
                # callback and this event when invoked via
                # `Simulator::Initialize` from `call_leaf_system_overrides`,
                # even when we explicitly say not to publish at initialize.
                self.called_publish = True

            def DoHasDirectFeedthrough(self, input_port, output_port):
                # Test inputs.
                test.assertEqual(input_port, 0)
                test.assertEqual(output_port, 0)
                # Call base method to ensure we do not get recursion.
                with catch_drake_warnings(expected_count=1):
                    base_return = LeafSystem_[T].DoHasDirectFeedthrough(
                        self, input_port, output_port)
                test.assertTrue(base_return is None)
                # Return custom methods.
                self.called_feedthrough = True
                return False

            def DoCalcTimeDerivatives(self, context, derivatives):
                # Note:  Don't call base method here; it would abort because
                # derivatives.size() != 0.
                test.assertEqual(derivatives.get_vector().size(), 2)
                self.called_continuous = True

            def DoCalcDiscreteVariableUpdates(
                    self, context, events, discrete_state):
                # Call base method to ensure we do not get recursion.
                LeafSystem_[T].DoCalcDiscreteVariableUpdates(
                    self, context, events, discrete_state)
                self.called_discrete = True

            def DoGetWitnessFunctions(self, context):
                self.called_getwitness = True
                return [self.witness, self.reset_witness]

            def _on_initialize(self, context, event):
                test.assertIsInstance(context, Context_[T])
                test.assertIsInstance(event, PublishEvent_[T])
                test.assertFalse(self.called_initialize)
                self.called_initialize = True

            def _on_per_step(self, context, event):
                test.assertIsInstance(context, Context_[T])
                test.assertIsInstance(event, PublishEvent_[T])
                self.called_per_step = True

            def _on_periodic(self, context, event):
                test.assertIsInstance(context, Context_[T])
                test.assertIsInstance(event, PublishEvent_[T])
                test.assertFalse(self.called_periodic)
                self.called_periodic = True

            def _witness(self, context):
                test.assertIsInstance(context, Context_[T])
                self.called_witness = True
                return 1.0

            def _guard(self, context):
                test.assertIsInstance(context, Context_[T])
                self.called_guard = True
                return context.get_time() - 0.5

            def _reset(self, context, event, state):
                test.assertIsInstance(context, Context_[T])
                test.assertIsInstance(event, UnrestrictedUpdateEvent_[T])
                test.assertIsInstance(state, State_[T])
                self.called_reset = True

        system = TrivialSystem()
        self.assertFalse(system.called_publish)
        self.assertFalse(system.called_feedthrough)
        self.assertFalse(system.called_continuous)
        self.assertFalse(system.called_discrete)
        self.assertFalse(system.called_initialize)
        results = call_leaf_system_overrides(system)
        self.assertTrue(system.called_publish)
        self.assertTrue(system.called_feedthrough)
        self.assertFalse(results["has_direct_feedthrough"])
        self.assertTrue(system.called_continuous)
        self.assertTrue(system.called_discrete)
        self.assertTrue(system.called_initialize)
        self.assertEqual(results["discrete_next_t"], 1.0)

        self.assertFalse(system.HasAnyDirectFeedthrough())
        self.assertFalse(system.HasDirectFeedthrough(output_port=0))
        self.assertFalse(
            system.HasDirectFeedthrough(input_port=0, output_port=0))

        # Test explicit calls.
        system = TrivialSystem()
        context = system.CreateDefaultContext()
        system.Publish(context)
        self.assertTrue(system.called_publish)
        context_update = context.Clone()
        system.CalcTimeDerivatives(
            context, context_update.get_mutable_continuous_state())
        self.assertTrue(system.called_continuous)
        witnesses = system.GetWitnessFunctions(context)
        self.assertEqual(len(witnesses), 2)

        # Test per-step, periodic, and witness call backs
        system = TrivialSystem()
        simulator = Simulator_[T](system)
        simulator.get_mutable_context().SetAccuracy(0.1)
        # Stepping to 0.99 so that we get exactly one periodic event.
        simulator.AdvanceTo(0.99)
        self.assertTrue(system.called_per_step)
        self.assertTrue(system.called_periodic)
        self.assertTrue(system.called_getwitness)
        self.assertTrue(system.called_witness)
        self.assertTrue(system.called_guard)
        self.assertTrue(system.called_reset)

    def test_deprecated_protected_aliases(self):
        """Tests a subset of protected aliases, pursuant to #9651."""

        class OldSystem(LeafSystem):
            def __init__(self):
                LeafSystem.__init__(self)
                self.called_publish = False
                # Check a non-overridable method
                with catch_drake_warnings(expected_count=1):
                    self._DeclareVectorInputPort("x", BasicVector(1))

            def _DoPublish(self, context, events):
                self.called_publish = True

        # Ensure old overrides are still used
        system = OldSystem()
        context = system.CreateDefaultContext()
        with catch_drake_warnings(expected_count=1):
            system.Publish(context)
        self.assertTrue(system.called_publish)

        # Ensure documentation includes the deprecation message.
        self.assertIn("deprecated", LeafSystem._DoPublish.__doc__)
        # This will warn both on (a) calling the method and (b) on the
        # invocation of the override.
        with catch_drake_warnings(expected_count=2):
            LeafSystem._DoPublish(system, context, [])

        class AccidentallyBothSystem(LeafSystem):
            def __init__(self):
                LeafSystem.__init__(self)
                self.called_old_publish = False
                self.called_new_publish = False

            def DoPublish(self, context, events):
                self.called_new_publish = True

            def _DoPublish(self, context, events):
                self.called_old_publish = True

        system = AccidentallyBothSystem()
        context = system.CreateDefaultContext()
        # This will trigger no deprecations, as the newer publish is called.
        system.Publish(context)
        self.assertTrue(system.called_new_publish)
        self.assertFalse(system.called_old_publish)

    def test_vector_system_overrides(self):
        map(self._check_vector_system_overrides,
            (float, AutoDiffXd, Expression))

    def _check_vector_system_overrides(self, T):
        dt = 0.5
        for is_discrete in [False, True]:
            system = CustomVectorSystem_[T](is_discrete)
            context = system.CreateDefaultContext()

            u = np.array([1.])
            context.FixInputPort(0, BasicVector_[T](u))

            # Dispatch virtual calls from C++.
            output = call_vector_system_overrides(
                system, context, is_discrete, dt)
            self.assertTrue(system.HasAnyDirectFeedthrough())

            # Check call order.
            update_type = is_discrete and "discrete" or "continuous"
            expected = [update_type, "feedthrough", "output", "feedthrough"]
            if T == Expression:
                # TODO(eric.cousineau): Why does this happen???
                expected = [update_type, "output", "feedthrough"]
            self.assertEqual(system.has_called, expected)

            # Check values.
            state = context.get_state()
            x = (is_discrete and state.get_discrete_state()
                 or state.get_continuous_state()).get_vector().get_value()

            x0 = [0., 0.]
            c = is_discrete and 2 or 1*dt
            x_expected = x0 + c*u
            if T != Expression:
                # TODO(eric.cousineau): Fix for symbolic.
                self.assertTrue(np.allclose(x, x_expected))

            # Check output.
            y_expected = np.hstack([u, x])
            y = output.get_vector_data(0).get_value()
            if T != Expression:
                # TODO(eric.cousineau): Fix for symbolic.
                self.assertTrue(np.allclose(y, y_expected))

    def test_context_api(self):
        # Capture miscellaneous functions not yet tested.
        model_value = AbstractValue.Make("Hello")
        model_vector = BasicVector([1., 2.])

        class TrivialSystem(LeafSystem):
            def __init__(self):
                LeafSystem.__init__(self)
                self.DeclareContinuousState(1)
                self.DeclareDiscreteState(2)
                self.DeclareAbstractState(model_value.Clone())
                self.DeclareAbstractParameter(model_value.Clone())
                self.DeclareNumericParameter(model_vector.Clone())

        system = TrivialSystem()
        context = system.CreateDefaultContext()
        self.assertTrue(
            context.get_state() is context.get_mutable_state())
        self.assertEqual(context.num_continuous_states(), 1)
        self.assertTrue(
            context.get_continuous_state_vector() is
            context.get_mutable_continuous_state_vector())
        self.assertEqual(context.num_discrete_state_groups(), 1)
        with catch_drake_warnings(expected_count=1):
            context.get_num_discrete_state_groups()
        self.assertTrue(
            context.get_discrete_state_vector() is
            context.get_mutable_discrete_state_vector())
        self.assertTrue(
            context.get_discrete_state(0) is
            context.get_discrete_state_vector())
        self.assertTrue(
            context.get_discrete_state(0) is
            context.get_discrete_state().get_vector(0))
        self.assertTrue(
            context.get_mutable_discrete_state(0) is
            context.get_mutable_discrete_state_vector())
        self.assertTrue(
            context.get_mutable_discrete_state(0) is
            context.get_mutable_discrete_state().get_vector(0))
        self.assertEqual(context.num_abstract_states(), 1)
        with catch_drake_warnings(expected_count=1):
            context.get_num_abstract_states()
        self.assertTrue(
            context.get_abstract_state() is
            context.get_mutable_abstract_state())
        self.assertTrue(
            context.get_abstract_state(0) is
            context.get_mutable_abstract_state(0))
        self.assertEqual(
            context.get_abstract_state(0).get_value(), model_value.get_value())

        # Check abstract state API (also test AbstractValues).
        values = context.get_abstract_state()
        self.assertEqual(values.size(), 1)
        self.assertEqual(
            values.get_value(0).get_value(), model_value.get_value())
        self.assertEqual(
            values.get_mutable_value(0).get_value(), model_value.get_value())
        values.SetFrom(values.Clone())
        with catch_drake_warnings(expected_count=1):
            values.CopyFrom(values.Clone())

        # Check parameter accessors.
        self.assertEqual(system.num_abstract_parameters(), 1)
        self.assertEqual(
            context.get_abstract_parameter(index=0).get_value(),
            model_value.get_value())
        self.assertEqual(system.num_numeric_parameter_groups(), 1)
        np.testing.assert_equal(
            context.get_numeric_parameter(index=0).get_value(),
            model_vector.get_value())

        # Check diagram context accessors.
        builder = DiagramBuilder()
        builder.AddSystem(system)
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        # Existence check.
        self.assertIsNot(
            diagram.GetMutableSubsystemState(system, context), None)
        subcontext = diagram.GetMutableSubsystemContext(system, context)
        self.assertIsNot(subcontext, None)
        self.assertIs(
            diagram.GetSubsystemContext(system, context), subcontext)

    def test_continuous_state_api(self):
        # N.B. Since this has trivial operations, we can test all scalar types.
        for T in [float, AutoDiffXd, Expression]:

            class TrivialSystem(LeafSystem_[T]):
                def __init__(self, index):
                    LeafSystem_[T].__init__(self)
                    num_q = 2
                    num_v = 1
                    num_z = 3
                    num_state = num_q + num_v + num_z
                    if index == 0:
                        self.DeclareContinuousState(
                            num_state_variables=num_state)
                    elif index == 1:
                        self.DeclareContinuousState(
                            num_q=num_q, num_v=num_v, num_z=num_z)
                    elif index == 2:
                        self.DeclareContinuousState(
                            BasicVector_[T](num_state))
                    elif index == 3:
                        self.DeclareContinuousState(
                            BasicVector_[T](num_state),
                            num_q=num_q, num_v=num_v, num_z=num_z)

                def DoCalcTimeDerivatives(self, context, derivatives):
                    derivatives.get_mutable_vector().SetZero()

            for index in range(4):
                system = TrivialSystem(index)
                context = system.CreateDefaultContext()
                self.assertEqual(
                    context.get_continuous_state_vector().size(), 6)
                self.assertEqual(system.AllocateTimeDerivatives().size(), 6)
                self.assertEqual(system.EvalTimeDerivatives(context).size(), 6)

    def test_discrete_state_api(self):
        # N.B. Since this has trivial operations, we can test all scalar types.
        for T in [float, AutoDiffXd, Expression]:

            class TrivialSystem(LeafSystem_[T]):
                def __init__(self, index):
                    LeafSystem_[T].__init__(self)
                    num_states = 3
                    if index == 0:
                        self.DeclareDiscreteState(
                            num_state_variables=num_states)
                    elif index == 1:
                        self.DeclareDiscreteState([1, 2, 3])
                    elif index == 2:
                        self.DeclareDiscreteState(
                            BasicVector_[T](num_states))

            for index in range(3):
                system = TrivialSystem(index)
                context = system.CreateDefaultContext()
                self.assertEqual(
                    context.get_discrete_state(0).size(), 3)

    def test_abstract_io_port(self):
        test = self
        # N.B. Since this has trivial operations, we can test all scalar types.
        for T in [float, AutoDiffXd, Expression]:
            default_value = ("default", T(0.))
            expected_input_value = ("input", T(np.pi))
            expected_output_value = ("output", 2*T(np.pi))

            class CustomAbstractSystem(LeafSystem_[T]):
                def __init__(self):
                    LeafSystem_[T].__init__(self)
                    self.input_port = self.DeclareAbstractInputPort(
                        "in", AbstractValue.Make(default_value))
                    self.output_port = self.DeclareAbstractOutputPort(
                        "out",
                        lambda: AbstractValue.Make(default_value),
                        self.DoCalcAbstractOutput)

                def DoCalcAbstractOutput(self, context, y_data):
                    input_value = self.EvalAbstractInput(
                        context, 0).get_value()
                    # The allocator function will populate the output with
                    # the "input"
                    test.assertTupleEqual(input_value, expected_input_value)
                    y_data.set_value(expected_output_value)
                    test.assertTupleEqual(y_data.get_value(),
                                          expected_output_value)

            system = CustomAbstractSystem()
            context = system.CreateDefaultContext()

            self.assertEqual(context.num_input_ports(), 1)
            context.FixInputPort(0, AbstractValue.Make(expected_input_value))
            output = system.AllocateOutput()
            self.assertEqual(output.num_ports(), 1)
            with catch_drake_warnings(expected_count=1):
                output.get_num_ports()
            system.CalcOutput(context, output)
            value = output.get_data(0)
            self.assertEqual(value.get_value(), expected_output_value)

    def test_deprecated_abstract_input_port(self):
        """This test case confirms that the deprecated API for abstract input ports
        continues to operate correctly, until such a time as we remove it.  For
        an example of non-deprecated APIs to use abstract input ports, see the
        test_abstract_io_port case, above.
        """
        test = self

        # A system that takes a Value[object] on its input, and parses the
        # input value's first element to a float on its output.
        class ParseFloatSystem(LeafSystem_[float]):
            def __init__(self):
                LeafSystem_[float].__init__(self)
                with catch_drake_warnings(expected_count=1):
                    self.DeclareAbstractInputPort("in")
                self.DeclareVectorOutputPort("out", BasicVector(1), self._Out)

            def _Out(self, context, y_data):
                py_obj = self.EvalAbstractInput(context, 0).get_value()[0]
                y_data.SetAtIndex(0, float(py_obj))

        system = ParseFloatSystem()
        context = system.CreateDefaultContext()
        output = system.AllocateOutput()
        context.FixInputPort(0, AbstractValue.Make(["22.2"]))
        system.CalcOutput(context, output)
        self.assertEqual(output.get_vector_data(0).GetAtIndex(0), 22.2)
