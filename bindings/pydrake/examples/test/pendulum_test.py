#!/usr/bin/env python

from __future__ import print_function

import copy
import unittest
import numpy as np

import pydrake.systems.framework as framework
from pydrake.examples.pendulum import (
    PendulumPlant,
    )
from pydrake.systems.analysis import (
    Simulator
    )
from pydrake.systems.primitives import (
    ConstantVectorSource,
    )


class TestPendulum(unittest.TestCase):
    def test_simulation(self):
        # Basic constant-torque pendulum simulation.

        builder = framework.DiagramBuilder()

        pendulum = builder.AddSystem(PendulumPlant())
        source = builder.AddSystem(ConstantVectorSource([1.0]))

        builder.Connect(source.get_output_port(0),
                        pendulum.get_input_port(0))

        diagram = builder.Build()
        simulator = Simulator(diagram)
        simulator.Initialize()

        # TODO(russt): Clean up state vector access below.
        state = simulator.get_mutable_context().get_mutable_state()\
                         .get_mutable_continuous_state().get_mutable_vector()

        initial_state = np.array([1.0, 0.0])
        state.SetFromVector(initial_state)

        simulator.StepTo(1.0)

        self.assertFalse((state.CopyToVector() == initial_state).all())


if __name__ == '__main__':
    unittest.main()
