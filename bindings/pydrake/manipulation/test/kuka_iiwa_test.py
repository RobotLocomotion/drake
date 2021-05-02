# -*- coding: utf-8 -*-

import pydrake.manipulation.kuka_iiwa as mut

import unittest
import numpy as np

from pydrake.systems.framework import InputPort, OutputPort


class TestKukaIiwa(unittest.TestCase):
    def test_kuka_iiwa_lcm(self):
        command_rec = mut.IiwaCommandReceiver()
        self.assertIsInstance(
            command_rec.get_message_input_port(), InputPort)
        self.assertIsInstance(
            command_rec.get_position_measured_input_port(), InputPort)
        self.assertIsInstance(
            command_rec.get_commanded_position_output_port(), OutputPort)
        self.assertIsInstance(
            command_rec.get_commanded_torque_output_port(), OutputPort)

        command_send = mut.IiwaCommandSender()
        self.assertIsInstance(
            command_send.get_position_input_port(), InputPort)
        self.assertIsInstance(
            command_send.get_torque_input_port(), InputPort)

        status_rec = mut.IiwaStatusReceiver()
        self.assertIsInstance(
            status_rec.get_position_commanded_output_port(), OutputPort)
        self.assertIsInstance(
            status_rec.get_position_measured_output_port(), OutputPort)
        self.assertIsInstance(
            status_rec.get_velocity_estimated_output_port(), OutputPort)
        self.assertIsInstance(
            status_rec.get_torque_commanded_output_port(), OutputPort)
        self.assertIsInstance(
            status_rec.get_torque_measured_output_port(), OutputPort)
        self.assertIsInstance(
            status_rec.get_torque_external_output_port(), OutputPort)

        status_send = mut.IiwaStatusSender()
        self.assertIsInstance(
            status_send.get_position_commanded_input_port(), InputPort)
        self.assertIsInstance(
            status_send.get_position_measured_input_port(), InputPort)
        self.assertIsInstance(
            status_send.get_velocity_estimated_input_port(), InputPort)
        self.assertIsInstance(
            status_send.get_torque_commanded_input_port(), InputPort)
        self.assertIsInstance(
            status_send.get_torque_measured_input_port(), InputPort)
        self.assertIsInstance(
            status_send.get_torque_external_input_port(), InputPort)

    def test_kuka_iiwa_api(self):
        self.assertEqual(mut.get_iiwa_max_joint_velocities().shape, (7,))
