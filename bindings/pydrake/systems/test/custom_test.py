#!/usr/bin/env python
# -*- coding: utf8 -*-

from __future__ import print_function

import copy
import unittest
import numpy as np

from pydrake.systems.analysis import (
    Simulator,
    )
from pydrake.systems.framework import (
    BasicVector,
    DiagramBuilder,
    LeafSystem,
    PortDataType,
    VectorSystem,
    )
from pydrake.systems.primitives import (
    ZeroOrderHold,
    )

from pydrake.systems.test.test_util import (
    call_leaf_system_overrides,
    call_vector_system_overrides,
    )


def noop(*args, **kwargs):
    # When a callback is required for an interface, but not useful for testing.
    pass


class CustomAdder(LeafSystem):
    # Reimplements `Adder`.
    def __init__(self, num_inputs, size):
        LeafSystem.__init__(self)
        for i in xrange(num_inputs):
            self._DeclareInputPort(PortDataType.kVectorValued, size)
        self._DeclareVectorOutputPort(BasicVector(size), self._calc_sum)

    def _calc_sum(self, context, sum_data):
        # @note This will NOT work if the scalar type is AutoDiff or symbolic,
        # since they are not stored densely.
        sum = sum_data.get_mutable_value()
        sum[:] = 0
        for i in xrange(context.get_num_input_ports()):
            input_vector = self.EvalVectorInput(context, i)
            sum += input_vector.get_value()


class CustomVectorSystem(VectorSystem):
    def __init__(self, is_discrete):
        # VectorSystem only supports pure Continuous or pure Discrete.
        # Dimensions:
        #   1 Input, 2 States, 3 Outputs.
        VectorSystem.__init__(self, 1, 3)
        self._is_discrete = is_discrete
        if self._is_discrete:
            self._DeclareDiscreteState(2)
        else:
            self._DeclareContinuousState(2)
        # Record calls for testing.
        self.has_called = []

    def _DoCalcVectorOutput(self, context, u, x, y):
        y[:] = np.hstack([u, x])
        self.has_called.append("output")

    def _DoCalcVectorTimeDerivatives(self, context, u, x, x_dot):
        x_dot[:] = x + u
        self.has_called.append("continuous")

    def _DoCalcVectorDiscreteVariableUpdates(self, context, u, x, x_n):
        x_n[:] = x + 2*u
        self.has_called.append("discrete")


class TestCustom(unittest.TestCase):
    def _create_adder_system(self):
        system = CustomAdder(2, 3)
        return system

    def _fix_adder_inputs(self, context):
        self.assertEquals(context.get_num_input_ports(), 2)
        context.FixInputPort(0, BasicVector([1, 2, 3]))
        context.FixInputPort(1, BasicVector([4, 5, 6]))

    def test_adder_execution(self):
        system = self._create_adder_system()
        context = system.CreateDefaultContext()
        self._fix_adder_inputs(context)
        output = system.AllocateOutput(context)
        self.assertEquals(output.get_num_ports(), 1)
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
        simulator.StepTo(1)
        # Ensure that we have the outputs we want.
        value = (diagram.GetMutableSubsystemContext(zoh, context)
                 .get_discrete_state_vector().get_value())
        self.assertTrue(np.allclose([5, 7, 9], value))

    def test_leaf_system_overrides(self):
        test = self

        class TrivialSystem(LeafSystem):
            def __init__(self):
                LeafSystem.__init__(self)
                self.called_publish = False
                self.called_feedthrough = False
                self.called_discrete = False
                # Ensure we have desired overloads.
                self._DeclarePeriodicPublish(0.1)
                self._DeclarePeriodicPublish(0.1, 0)
                self._DeclarePeriodicPublish(period_sec=0.1, offset_sec=0.)
                self._DeclarePeriodicDiscreteUpdate(
                    period_sec=0.1, offset_sec=0.)
                self._DeclareDiscreteState(1)
                # Ensure that we have inputs / outputs to call direct
                # feedthrough.
                self._DeclareInputPort(PortDataType.kVectorValued, 1)
                self._DeclareVectorOutputPort(BasicVector(1), noop)

            def _DoPublish(self, context, events):
                # Call base method to ensure we do not get recursion.
                LeafSystem._DoPublish(self, context, events)
                self.called_publish = True

            def _DoHasDirectFeedthrough(self, input_port, output_port):
                # Test inputs.
                test.assertEquals(input_port, 0)
                test.assertEquals(output_port, 0)
                # Call base method to ensure we do not get recursion.
                base_return = LeafSystem._DoHasDirectFeedthrough(
                    self, input_port, output_port)
                test.assertTrue(base_return is None)
                # Return custom methods.
                self.called_feedthrough = True
                return False

            def _DoCalcDiscreteVariableUpdates(
                    self, context, events, discrete_state):
                # Call base method to ensure we do not get recursion.
                LeafSystem._DoCalcDiscreteVariableUpdates(
                    self, context, events, discrete_state)
                self.called_discrete = True

        system = TrivialSystem()
        self.assertFalse(system.called_publish)
        self.assertFalse(system.called_feedthrough)
        self.assertFalse(system.called_discrete)
        results = call_leaf_system_overrides(system)
        self.assertTrue(system.called_publish)
        self.assertTrue(system.called_feedthrough)
        self.assertFalse(results["has_direct_feedthrough"])
        self.assertTrue(system.called_discrete)
        self.assertEquals(results["discrete_next_t"], 0.1)

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

            # Check call order.
            update_type = is_discrete and "discrete" or "continuous"
            assert system.has_called == [update_type, "output"]

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

        class TrivialSystem(LeafSystem):
            def __init__(self):
                LeafSystem.__init__(self)
                self._DeclareContinuousState(1)
                self._DeclareDiscreteState(2)

        system = TrivialSystem()
        context = system.CreateDefaultContext()
        self.assertTrue(
            context.get_state() is context.get_mutable_state())
        self.assertTrue(
            context.get_continuous_state_vector() is
            context.get_mutable_continuous_state_vector())
        self.assertTrue(
            context.get_discrete_state_vector() is
            context.get_mutable_discrete_state_vector())

        builder = DiagramBuilder()
        builder.AddSystem(system)
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        # Existence check.
        self.assertTrue(
            diagram.GetMutableSubsystemState(system, context) is not None)
        self.assertTrue(
            diagram.GetMutableSubsystemContext(system, context) is not None)

    def test_continuous_state_api(self):

        class TrivialSystem(LeafSystem):
            def __init__(self, index):
                LeafSystem.__init__(self)
                num_q = 2
                num_v = 1
                num_z = 3
                num_state = num_q + num_v + num_z
                if index == 0:
                    self._DeclareContinuousState(num_state_variables=num_state)
                elif index == 1:
                    self._DeclareContinuousState(
                        num_q=num_q, num_v=num_v, num_z=num_z)
                elif index == 2:
                    self._DeclareContinuousState(BasicVector(num_state))
                elif index == 3:
                    self._DeclareContinuousState(
                        BasicVector(num_state),
                        num_q=num_q, num_v=num_v, num_z=num_z)

        for index in range(4):
            system = TrivialSystem(index)
            context = system.CreateDefaultContext()
            self.assertEquals(context.get_continuous_state_vector().size(), 6)


if __name__ == '__main__':
    unittest.main()
