from __future__ import print_function

import unittest

from pydrake.examples.rimless_wheel import (
    RimlessWheelParams, RimlessWheel, RimlessWheelContinuousState
    )
from pydrake.systems.analysis import (
    Simulator
    )


class TestRimlessWheel(unittest.TestCase):
    def test_params(self):
        params = RimlessWheelParams()
        params.set_mass(1.)
        params.set_length(2.)
        params.set_gravity(4.)
        params.set_number_of_spokes(7)
        params.set_slope(.15)
        self.assertEqual(params.mass(), 1.)
        self.assertEqual(params.length(), 2.)
        self.assertEqual(params.gravity(), 4.)
        self.assertEqual(params.number_of_spokes(), 7)
        self.assertEqual(params.slope(), .15)

    def test_state(self):
        state = RimlessWheelContinuousState()
        state.set_theta(1.)
        state.set_thetadot(2.)
        self.assertEqual(state.theta(), 1.)
        self.assertEqual(state.thetadot(), 2.)

    def test_simulation(self):
        # Basic rimless_wheel simulation.
        rimless_wheel = RimlessWheel()

        # Create the simulator.
        simulator = Simulator(rimless_wheel)
        context = simulator.get_mutable_context()
        context.SetAccuracy(1e-8)

        # Set the initial state.
        state = context.get_mutable_continuous_state_vector()
        state.set_theta(0.)
        state.set_thetadot(4.)

        # Simulate (and make sure the state actually changes).
        initial_state = state.CopyToVector()
        simulator.AdvanceTo(1.0)
        self.assertFalse((state.CopyToVector() == initial_state).any())
