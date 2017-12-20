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
    )
from pydrake.systems.primitives import (
    ZeroOrderHold,
    )


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


class TestCustom(unittest.TestCase):
    def _create_system(self):
        system = CustomAdder(2, 3)
        return system

    def _fix_inputs(self, context):
        self.assertEquals(context.get_num_input_ports(), 2)
        context.FixInputPort(0, BasicVector([1, 2, 3]))
        context.FixInputPort(1, BasicVector([4, 5, 6]))

    def test_execution(self):
        system = self._create_system()
        context = system.CreateDefaultContext()
        self._fix_inputs(context)
        output = system.AllocateOutput(context)
        self.assertEquals(output.get_num_ports(), 1)
        system.CalcOutput(context, output)
        value = output.get_vector_data(0).get_value()
        self.assertTrue(np.allclose([5, 7, 9], value))

    def test_simulation(self):
        builder = DiagramBuilder()
        adder = builder.AddSystem(self._create_system())
        adder.set_name("custom_adder")
        # Add ZOH so we can easily extract state.
        zoh = builder.AddSystem(ZeroOrderHold(0.1, 3))
        zoh.set_name("zoh")

        builder.ExportInput(adder.get_input_port(0))
        builder.ExportInput(adder.get_input_port(1))
        builder.Connect(adder.get_output_port(0), zoh.get_input_port(0))
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        self._fix_inputs(context)

        simulator = Simulator(diagram, context)
        simulator.Initialize()
        simulator.StepTo(1)
        # Ensure that we have the outputs we want.
        state = diagram.GetMutableSubsystemState(zoh, context)
        value = state.get_discrete_state().get_data()[0].get_value()
        self.assertTrue(np.allclose([5, 7, 9], value))


if __name__ == '__main__':
    unittest.main()
