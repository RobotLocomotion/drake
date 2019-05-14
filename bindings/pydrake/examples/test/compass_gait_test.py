from __future__ import print_function

import unittest

from pydrake.examples.compass_gait import (
    CompassGaitParams, CompassGait, CompassGaitContinuousState
    )
from pydrake.systems.analysis import (
    Simulator
    )


class TestCompassGait(unittest.TestCase):
    def test_params(self):
        params = CompassGaitParams()
        params.set_mass_hip(1.)
        params.set_mass_leg(2.)
        params.set_length_leg(3.)
        params.set_center_of_mass_leg(4.)
        params.set_gravity(5.)
        params.set_slope(.15)
        self.assertEqual(params.mass_hip(), 1.)
        self.assertEqual(params.mass_leg(), 2.)
        self.assertEqual(params.length_leg(), 3.)
        self.assertEqual(params.center_of_mass_leg(), 4.)
        self.assertEqual(params.gravity(), 5.)
        self.assertEqual(params.slope(), .15)

    def test_state(self):
        state = CompassGaitContinuousState()
        state.set_stance(1.)
        state.set_swing(2.)
        state.set_stancedot(3.)
        state.set_swingdot(4.)
        self.assertEqual(state.stance(), 1.)
        self.assertEqual(state.swing(), 2.)
        self.assertEqual(state.stancedot(), 3.)
        self.assertEqual(state.swingdot(), 4.)

    def test_simulation(self):
        # Basic compass_gait simulation.
        compass_gait = CompassGait()

        # Create the simulator.
        simulator = Simulator(compass_gait)
        context = simulator.get_mutable_context()
        context.SetAccuracy(1e-8)

        # Zero the input
        context.FixInputPort(compass_gait.get_input_port(0).get_index(), [0.0])

        # Set the initial state.
        state = context.get_mutable_continuous_state_vector()
        state.set_stance(0.)
        state.set_swing(0.)
        state.set_stancedot(0.4)
        state.set_swingdot(-2.0)

        # Simulate (and make sure the state actually changes).
        initial_state = state.CopyToVector()
        simulator.AdvanceTo(1.0)
        self.assertFalse((state.CopyToVector() == initial_state).any())
