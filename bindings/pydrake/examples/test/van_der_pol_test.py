from __future__ import print_function

import unittest

from pydrake.examples.van_der_pol import VanDerPolOscillator
from pydrake.systems.analysis import Simulator


class TestVanDerPol(unittest.TestCase):
    def test_simulation(self):
        van_der_pol = VanDerPolOscillator()

        # Create the simulator.
        simulator = Simulator(van_der_pol)
        context = simulator.get_mutable_context()

        # Set the initial state.
        state = context.get_mutable_continuous_state_vector()
        state.SetFromVector([0., 2.])

        # Simulate (and make sure the state actually changes).
        initial_state = state.CopyToVector()
        simulator.StepTo(1.0)
        self.assertFalse((state.CopyToVector() == initial_state).any())
