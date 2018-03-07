# -*- coding: utf-8 -*-

from __future__ import print_function

import copy

import unittest
import numpy as np

from pydrake.systems.analysis import (
    Simulator,
    )
from pydrake.systems.framework import (
    BasicVector,
    Diagram,
    DiagramBuilder,
    )
from pydrake.systems.primitives import (
    Adder,
    AffineSystem,
    ConstantVectorSource,
    Integrator,
    LinearSystem,
    SignalLogger,
    )


class TestGeneral(unittest.TestCase):
    def test_simulator_ctor(self):
        # Create simple system.
        system = ConstantVectorSource([1])

        def check_output(context):
            # Check number of output ports and value for a given context.
            output = system.AllocateOutput(context)
            self.assertEquals(output.get_num_ports(), 1)
            system.CalcOutput(context, output)
            value = output.get_vector_data(0).get_value()
            self.assertTrue(np.allclose([1], value))

        # Create simulator with basic constructor.
        simulator = Simulator(system)
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
        # @note `simulator` now owns `context`.
        simulator = Simulator(system, context)
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

        builder.ExportInput(adder0.get_input_port(0))
        builder.ExportInput(adder0.get_input_port(1))
        builder.ExportInput(adder1.get_input_port(1))
        builder.ExportOutput(integrator.get_output_port(0))

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
        input0 = BasicVector([0.1, 0.2, 0.3])
        context.FixInputPort(0, input0)
        input1 = BasicVector([0.02, 0.03, 0.04])
        context.FixInputPort(1, input1)
        input2 = BasicVector([0.003, 0.004, 0.005])
        context.FixInputPort(2, input2)

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
