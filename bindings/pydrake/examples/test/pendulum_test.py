#!/usr/bin/env python

from __future__ import print_function

import copy
import unittest
import numpy as np

import pydrake.systems.framework as framework
from pydrake.examples.pendulum import (
    PendulumInput, PendulumParams, PendulumPlant, PendulumState
    )
from pydrake.systems.analysis import (
    Simulator
    )


class TestPendulum(unittest.TestCase):
    def test_input(self):
        input = PendulumInput()
        input.set_tau(1.)
        self.assertEqual(input.tau(), 1.)

    def test_params(self):
        params = PendulumParams()
        params.set_mass(1.)
        params.set_length(2.)
        params.set_damping(3.)
        params.set_gravity(4.)
        self.assertEqual(params.mass(), 1.)
        self.assertEqual(params.length(), 2.)
        self.assertEqual(params.damping(), 3.)
        self.assertEqual(params.gravity(), 4.)

    def test_state(self):
        state = PendulumState()
        state.set_theta(1.)
        state.set_thetadot(2.)
        self.assertEqual(state.theta(), 1.)
        self.assertEqual(state.thetadot(), 2.)

    def test_simulation(self):
        # Basic constant-torque pendulum simulation.
        pendulum = PendulumPlant()

        # Create the simulator.
        simulator = Simulator(pendulum)
        context = simulator.get_mutable_context()

        # Set an input torque.
        input = PendulumInput()
        input.set_tau(1.)
        context.FixInputPort(0, input)

        # Set the initial state.
        state = context.get_mutable_continuous_state_vector()
        state.set_theta(1.)
        state.set_thetadot(0.)

        # Simulate (and make sure the state actually changes).
        initial_state = state.CopyToVector()
        simulator.StepTo(1.0)
        self.assertFalse((state.CopyToVector() == initial_state).any())


if __name__ == '__main__':
    unittest.main()
