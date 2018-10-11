"""Provides example of running van_der_pol bindings.

To run from an installation, please execute:

    python {install_dir}/share/drake/examples/van_der_pol/van_der_pol_example.py  # noqa
"""

from __future__ import print_function

from pydrake.systems.analysis import Simulator

from van_der_pol import VanDerPolOscillator


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
print("Final state: {}".format(state.CopyToVector()))
