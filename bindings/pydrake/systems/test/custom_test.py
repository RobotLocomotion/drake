# -*- coding: utf-8 -*-

from __future__ import print_function

import copy
import unittest
import warnings
import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
from pydrake.systems.analysis import (
    Simulator,
    )
from pydrake.systems.framework import (
    AbstractParameterIndex,
    AbstractStateIndex,
    AbstractValue,
    BasicVector, BasicVector_,
    CacheIndex,
    Context,
    DependencyTicket,
    Diagram,
    DiagramBuilder,
    DiscreteStateIndex,
    InputPortIndex,
    LeafSystem, LeafSystem_,
    NumericParameterIndex,
    PortDataType,
    PublishEvent,
    State,
    TriggerType,
    UnrestrictedUpdateEvent,
    VectorSystem,
    WitnessFunctionDirection,
    kUseDefaultName,
    )
from pydrake.systems.primitives import (
    Adder,
    ZeroOrderHold,
    )

from pydrake.systems.test.test_util import (
    call_leaf_system_overrides,
    call_vector_system_overrides,
    )

from pydrake.common.test_utilities.deprecation import catch_drake_warnings


def noop(*args, **kwargs):
    # When a callback is required for an interface, but not useful for testing.
    pass


class CustomAdder(LeafSystem):
    # Reimplements `Adder`.
    def __init__(self, num_inputs, size):
        LeafSystem.__init__(self)
        for i in range(num_inputs):
            self.DeclareVectorInputPort(
                "input{}".format(i), BasicVector(size))
        self.DeclareVectorOutputPort("sum", BasicVector(size), self._calc_sum)

    def _calc_sum(self, context, sum_data):
        # @note This will NOT work if the scalar type is AutoDiff or symbolic,
        # since they are not stored densely.
        sum = sum_data.get_mutable_value()
        sum[:] = 0
        for i in range(context.num_input_ports()):
            input_vector = self.EvalVectorInput(context, i)
            sum += input_vector.get_value()


# TODO(eric.cousineau): Make this class work with custom scalar types once
# referencing with custom dtypes lands.
# WARNING: At present, dtype=object matrices are NOT well supported, and may
# produce unexpected results (e.g. references not actually being respected).


class CustomVectorSystem(VectorSystem):
    def __init__(self, is_discrete):
        # VectorSystem only supports pure Continuous or pure Discrete.
        # Dimensions:
        #   1 Input, 2 States, 3 Outputs.
        VectorSystem.__init__(self, 1, 3)
        self._is_discrete = is_discrete
        if self._is_discrete:
            self.DeclareDiscreteState(2)
        else:
            self.DeclareContinuousState(2)
        # Record calls for testing.
        self.has_called = []

    def DoCalcVectorOutput(self, context, u, x, y):
        y[:] = np.hstack([u, x])
        self.has_called.append("output")

    def DoCalcVectorTimeDerivatives(self, context, u, x, x_dot):
        x_dot[:] = x + u
        self.has_called.append("continuous")

    def DoCalcVectorDiscreteVariableUpdates(self, context, u, x, x_n):
        x_n[:] = x + 2*u
        self.has_called.append("discrete")


# Wraps `Adder`.
class CustomDiagram(Diagram):
    # N.B. The CustomDiagram is used to unit test the DiagramBuilder.BuildInto
    # method.  For pydrake users, this is not a good example.  The best way in
    # pydrake to create a Diagram is DiagramBuilder.Build (as seen in the test
    # case named test_adder_simulation).

    def __init__(self, num_inputs, size):
        Diagram.__init__(self)
        builder = DiagramBuilder()
        adder = Adder(num_inputs, size)
        builder.AddSystem(adder)
        builder.ExportOutput(adder.get_output_port(0))
        for i in range(num_inputs):
            builder.ExportInput(adder.get_input_port(i))
        builder.BuildInto(self)


class TestCustom(unittest.TestCase):
    def _create_adder_system(self):
        system = CustomAdder(2, 3)
        return system

    def _fix_adder_inputs(self, context):
        self.assertEqual(context.num_input_ports(), 2)
        context.FixInputPort(0, BasicVector([1, 2, 3]))
        context.FixInputPort(1, BasicVector([4, 5, 6]))

    def test_diagram_adder(self):
        system = CustomDiagram(2, 3)
        self.assertEqual(system.num_input_ports(), 2)
        self.assertEqual(system.get_input_port(0).size(), 3)
        self.assertEqual(system.num_output_ports(), 1)
        self.assertEqual(system.get_output_port(0).size(), 3)

    def test_adder_execution(self):
        system = self._create_adder_system()
        context = system.CreateDefaultContext()
        self.assertEqual(context.num_output_ports(), 1)
        self._fix_adder_inputs(context)
        output = system.AllocateOutput()
        self.assertEqual(output.num_ports(), 1)
        system.CalcOutput(context, output)
        value = output.get_vector_data(0).get_value()
        self.assertTrue(np.allclose([5, 7, 9], value))

    def test_adder_simulation(self):
        builder = DiagramBuilder()
        adder = builder.AddSystem(self._create_adder_system())
        adder.set_name("custom_adder")
        # Add ZOH so we can easily extract state.
        zoh = builder.AddSystem(ZeroOrderHold(0.1, 3))
        zoh.set_name("zoh")

        builder.ExportInput(adder.get_input_port(0))
        builder.ExportInput(adder.get_input_port(1))
        builder.Connect(adder.get_output_port(0), zoh.get_input_port(0))
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        self._fix_adder_inputs(context)

        simulator = Simulator(diagram, context)
        simulator.Initialize()
        simulator.AdvanceTo(1)
        # Ensure that we have the outputs we want.
        value = (diagram.GetMutableSubsystemContext(zoh, context)
                 .get_discrete_state_vector().get_value())
        self.assertTrue(np.allclose([5, 7, 9], value))

    def test_leaf_system_well_known_tickets(self):
        for func in [
                LeafSystem.accuracy_ticket,
                LeafSystem.all_input_ports_ticket,
                LeafSystem.all_parameters_ticket,
                LeafSystem.all_sources_ticket,
                LeafSystem.all_state_ticket,
                LeafSystem.configuration_ticket,
                LeafSystem.ke_ticket,
                LeafSystem.kinematics_ticket,
                LeafSystem.nothing_ticket,
                LeafSystem.pa_ticket,
                LeafSystem.pc_ticket,
                LeafSystem.pe_ticket,
                LeafSystem.pn_ticket,
                LeafSystem.pnc_ticket,
                LeafSystem.q_ticket,
                LeafSystem.time_ticket,
                LeafSystem.v_ticket,
                LeafSystem.xa_ticket,
                LeafSystem.xc_ticket,
                LeafSystem.xcdot_ticket,
                LeafSystem.xd_ticket,
                LeafSystem.z_ticket]:
            self.assertIsInstance(func(), DependencyTicket, func)

    def test_leaf_system_per_item_tickets(self):
        dut = LeafSystem()
        dut.DeclareAbstractParameter(AbstractValue.Make(1))
        dut.DeclareAbstractState(AbstractValue.Make(1))
        dut.DeclareDiscreteState(1)
        dut.DeclareVectorInputPort("u0", BasicVector(1))
        dut.DeclareNumericParameter(BasicVector(1))
        for func, arg in [
                (dut.abstract_parameter_ticket, AbstractParameterIndex(0)),
                (dut.abstract_state_ticket, AbstractStateIndex(0)),
                (dut.cache_entry_ticket, CacheIndex(0)),
                (dut.discrete_state_ticket, DiscreteStateIndex(0)),
                (dut.input_port_ticket, InputPortIndex(0)),
                (dut.numeric_parameter_ticket, NumericParameterIndex(0)),
                ]:
            self.assertIsInstance(func(arg), DependencyTicket, func)

    def test_all_leaf_system_overrides(self):
        test = self

        class TrivialSystem(LeafSystem):
            def __init__(self):
                LeafSystem.__init__(self)
                self.called_publish = False
                self.called_continuous = False
                self.called_discrete = False
                self.called_initialize = False
                self.called_per_step = False
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
                    event=PublishEvent(
                        trigger_type=TriggerType.kInitialization,
                        callback=self._on_initialize))
                self.DeclarePerStepEvent(
                    event=PublishEvent(
                        trigger_type=TriggerType.kPerStep,
                        callback=self._on_per_step))
                self.DeclarePeriodicEvent(
                    period_sec=1.0,
                    offset_sec=0.0,
                    event=PublishEvent(
                        trigger_type=TriggerType.kPeriodic,
                        callback=self._on_periodic))
                self.DeclareContinuousState(2)
                self.DeclareDiscreteState(1)
                # Ensure that we have inputs / outputs to call direct
                # feedthrough.
                self.DeclareInputPort(PortDataType.kVectorValued, 1)
                self.DeclareVectorInputPort(
                    name="test_input", model_vector=BasicVector(1),
                    random_type=None)
                self.DeclareVectorOutputPort(
                    "noop", BasicVector(1), noop,
                    prerequisites_of_calc=set([self.nothing_ticket()]))
                self.witness = self.MakeWitnessFunction(
                    "witness", WitnessFunctionDirection.kCrossesZero,
                    self._witness)
                self.reset_witness = self.MakeWitnessFunction(
                    "reset", WitnessFunctionDirection.kCrossesZero,
                    self._guard, UnrestrictedUpdateEvent(self._reset))

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

            def DoCalcTimeDerivatives(self, context, derivatives):
                # Note:  Don't call base method here; it would abort because
                # derivatives.size() != 0.
                test.assertEqual(derivatives.get_vector().size(), 2)
                self.called_continuous = True

            def DoCalcDiscreteVariableUpdates(
                    self, context, events, discrete_state):
                # Call base method to ensure we do not get recursion.
                LeafSystem.DoCalcDiscreteVariableUpdates(
                    self, context, events, discrete_state)
                self.called_discrete = True

            def DoGetWitnessFunctions(self, context):
                self.called_getwitness = True
                return [self.witness, self.reset_witness]

            def _on_initialize(self, context, event):
                test.assertIsInstance(context, Context)
                test.assertIsInstance(event, PublishEvent)
                test.assertFalse(self.called_initialize)
                self.called_initialize = True

            def _on_per_step(self, context, event):
                test.assertIsInstance(context, Context)
                test.assertIsInstance(event, PublishEvent)
                self.called_per_step = True

            def _on_periodic(self, context, event):
                test.assertIsInstance(context, Context)
                test.assertIsInstance(event, PublishEvent)
                test.assertFalse(self.called_periodic)
                self.called_periodic = True

            def _witness(self, context):
                test.assertIsInstance(context, Context)
                self.called_witness = True
                return 1.0

            def _guard(self, context):
                test.assertIsInstance(context, Context)
                self.called_guard = True
                return context.get_time() - 0.5

            def _reset(self, context, event, state):
                test.assertIsInstance(context, Context)
                test.assertIsInstance(event, UnrestrictedUpdateEvent)
                test.assertIsInstance(state, State)
                self.called_reset = True

        system = TrivialSystem()
        self.assertFalse(system.called_publish)
        self.assertFalse(system.called_continuous)
        self.assertFalse(system.called_discrete)
        self.assertFalse(system.called_initialize)
        results = call_leaf_system_overrides(system)
        self.assertTrue(system.called_publish)
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
            context=context,
            derivatives=context_update.get_mutable_continuous_state())
        self.assertTrue(system.called_continuous)
        witnesses = system.GetWitnessFunctions(context)
        self.assertEqual(len(witnesses), 2)
        system.CalcDiscreteVariableUpdates(
            context=context,
            discrete_state=context_update.get_mutable_discrete_state())
        self.assertTrue(system.called_discrete)

        # Test per-step, periodic, and witness call backs
        system = TrivialSystem()
        simulator = Simulator(system)
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
        dt = 0.5
        for is_discrete in [False, True]:
            system = CustomVectorSystem(is_discrete)
            context = system.CreateDefaultContext()

            u = np.array([1.])
            context.FixInputPort(0, BasicVector(u))

            # Dispatch virtual calls from C++.
            output = call_vector_system_overrides(
                system, context, is_discrete, dt)
            self.assertTrue(system.HasAnyDirectFeedthrough())

            # Check call order.
            update_type = is_discrete and "discrete" or "continuous"
            self.assertEqual(
                system.has_called,
                [update_type, "output"])

            # Check values.
            state = context.get_state()
            x = (is_discrete and state.get_discrete_state()
                 or state.get_continuous_state()).get_vector().get_value()

            x0 = [0., 0.]
            c = is_discrete and 2 or 1*dt
            x_expected = x0 + c*u
            self.assertTrue(np.allclose(x, x_expected))

            # Check output.
            y_expected = np.hstack([u, x])
            y = output.get_vector_data(0).get_value()
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
                self.assertEqual(system.AllocateDiscreteVariables().size(), 3)

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
                        self.DoCalcAbstractOutput,
                        prerequisites_of_calc=set([
                            self.input_port.ticket()]))

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
