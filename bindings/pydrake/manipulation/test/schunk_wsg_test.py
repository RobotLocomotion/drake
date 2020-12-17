# -*- coding: utf-8 -*-

import pydrake.manipulation.schunk_wsg as mut

import unittest
import numpy as np

from pydrake.systems.framework import InputPort, OutputPort
from pydrake.systems.primitives import MatrixGain


class TestSchunkWsg(unittest.TestCase):
    def test_schunk_wsg_controller(self):
        controller = mut.SchunkWsgPositionController(time_step=0.01,
                                                     kp_command=100.,
                                                     kd_command=1.,
                                                     kp_constraint=1000.,
                                                     kd_constraint=1.,
                                                     default_force_limit=37.0)

        self.assertIsInstance(
            controller.get_desired_position_input_port(), InputPort)
        self.assertIsInstance(
            controller.get_force_limit_input_port(), InputPort)
        self.assertIsInstance(
            controller.get_state_input_port(), InputPort)
        self.assertIsInstance(
            controller.get_generalized_force_output_port(), OutputPort)
        self.assertIsInstance(
            controller.get_grip_force_output_port(), OutputPort)

    def test_schunk_wsg_lcm(self):
        command_rec = mut.SchunkWsgCommandReceiver(initial_position=0.01,
                                                   initial_force=5.)
        self.assertIsInstance(
            command_rec.get_position_output_port(), OutputPort)
        self.assertIsInstance(
            command_rec.get_force_limit_output_port(), OutputPort)

        command_send = mut.SchunkWsgCommandSender(default_force_limit=37.0)
        self.assertIsInstance(
            command_send.get_position_input_port(), InputPort)
        self.assertIsInstance(
            command_send.get_force_limit_input_port(), InputPort)
        self.assertIsInstance(
            command_send.get_command_output_port(), OutputPort)

        status_rec = mut.SchunkWsgStatusReceiver()
        self.assertIsInstance(status_rec.get_status_input_port(), InputPort)
        self.assertIsInstance(status_rec.get_state_output_port(), OutputPort)
        self.assertIsInstance(status_rec.get_force_output_port(), OutputPort)

        status_send = mut.SchunkWsgStatusSender()
        self.assertIsInstance(status_send.get_state_input_port(), InputPort)
        self.assertIsInstance(status_send.get_force_input_port(), InputPort)

    def test_schunk_wsg_api(self):
        self.assertEqual(mut.GetSchunkWsgOpenPosition().shape, (2,))
        self.assertIsInstance(
            mut.MakeMultibodyStateToWsgStateSystem(), MatrixGain)
