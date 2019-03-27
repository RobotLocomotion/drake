from __future__ import print_function

import copy
import unittest
import numpy as np

import pydrake.systems.framework as framework
from pydrake.examples.acrobot import (
    AcrobotInput, AcrobotParams, AcrobotPlant, AcrobotState
    )
from pydrake.systems.analysis import (
    Simulator
    )


class TestAcrobot(unittest.TestCase):
    def test_input(self):
        input = AcrobotInput()
        input.set_tau(1.)
        self.assertEqual(input.tau(), 1.)

    def test_params(self):
        params = AcrobotParams()
        params.set_m1(1.)
        params.set_m2(2.)
        params.set_l1(3.)
        params.set_Ic1(4.)
        params.set_Ic2(5.)
        params.set_b1(6.)
        params.set_b2(7.)
        params.set_gravity(8.)
        self.assertEqual(params.m1(), 1.)
        self.assertEqual(params.m2(), 2.)
        self.assertEqual(params.l1(), 3.)
        self.assertEqual(params.Ic1(), 4.)
        self.assertEqual(params.Ic2(), 5.)
        self.assertEqual(params.b1(), 6.)
        self.assertEqual(params.b2(), 7.)
        self.assertEqual(params.gravity(), 8.)

    def test_state(self):
        state = AcrobotState()
        state.set_theta1(1.)
        state.set_theta1dot(2.)
        state.set_theta2(3.)
        state.set_theta2dot(4.)
        self.assertEqual(state.theta1(), 1.)
        self.assertEqual(state.theta1dot(), 2.)
        self.assertEqual(state.theta2(), 3.)
        self.assertEqual(state.theta2dot(), 4.)

    def test_simulation(self):
        # Basic constant-torque acrobot simulation.
        acrobot = AcrobotPlant()

        # Create the simulator.
        simulator = Simulator(acrobot)
        context = simulator.get_mutable_context()

        # Set an input torque.
        input = AcrobotInput()
        input.set_tau(1.)
        context.FixInputPort(0, input)

        # Set the initial state.
        state = context.get_mutable_continuous_state_vector()
        state.set_theta1(1.)
        state.set_theta1dot(0.)
        state.set_theta2(0.)
        state.set_theta2dot(0.)

        self.assertTrue(acrobot.DynamicsBiasTerm(context).shape == (2,))
        self.assertTrue(acrobot.MassMatrix(context).shape == (2, 2))
        initial_total_energy = acrobot.CalcPotentialEnergy(context) + \
            acrobot.CalcKineticEnergy(context)

        # Simulate (and make sure the state actually changes).
        initial_state = state.CopyToVector()
        simulator.AdvanceTo(1.0)

        self.assertLessEqual(acrobot.CalcPotentialEnergy(context) +
                             acrobot.CalcKineticEnergy(context),
                             initial_total_energy)
