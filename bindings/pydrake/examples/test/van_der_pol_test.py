import numpy as np
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
        simulator.AdvanceTo(1.0)
        self.assertFalse((state.CopyToVector() == initial_state).any())

    def test_ports(self):
        vdp = VanDerPolOscillator()

        self.assertTrue(vdp.get_position_output_port().get_index().is_valid())
        self.assertTrue(
            vdp.get_full_state_output_port().get_index().is_valid())

    def test_limit_cycle(self):
        cycle = VanDerPolOscillator.CalcLimitCycle()

        np.testing.assert_almost_equal(cycle[:, 0], cycle[:, -1], 2)
