# -*- coding: utf-8 -*-

import unittest

import pydrake.manipulation as mut
from pydrake.systems.framework import (
    InputPort,
    OutputPort,
)


class TestFrankaPanda(unittest.TestCase):
    def test_constants(self):
        self.assertEqual(mut.kPandaArmNumJoints, 7)

    def test_control_mode_bitwise_or(self):
        # Test that OR-ing control modes produces correct bit patterns
        # kPosition = 1, kVelocity = 2, kTorque = 4
        self.assertEqual(mut.PandaControlMode.kPosition, 1)
        self.assertEqual(mut.PandaControlMode.kVelocity, 2)
        self.assertEqual(mut.PandaControlMode.kTorque, 4)
        self.assertEqual(mut.PandaControlMode.kNone, 0)

        # Test OR combinations
        self.assertEqual(
            mut.PandaControlMode.kPosition | mut.PandaControlMode.kVelocity, 3
        )
        self.assertEqual(
            mut.PandaControlMode.kPosition | mut.PandaControlMode.kTorque, 5
        )
        self.assertEqual(
            mut.PandaControlMode.kVelocity | mut.PandaControlMode.kTorque, 6
        )
        self.assertEqual(
            mut.PandaControlMode.kPosition
            | mut.PandaControlMode.kVelocity
            | mut.PandaControlMode.kTorque,
            7,
        )

    def test_franka_panda_command_receiver(self):
        # Test with position and torque control mode
        control_mode = (
            mut.PandaControlMode.kPosition | mut.PandaControlMode.kTorque
        )
        command_rec = mut.PandaCommandReceiver(
            num_joints=mut.kPandaArmNumJoints, control_mode=control_mode
        )
        self.assertIsInstance(command_rec.get_message_input_port(), InputPort)
        self.assertIsInstance(
            command_rec.get_position_measured_input_port(), InputPort
        )
        self.assertIsInstance(
            command_rec.get_commanded_position_output_port(), OutputPort
        )
        self.assertIsInstance(
            command_rec.get_commanded_torque_output_port(), OutputPort
        )

        # Test with position, velocity, and torque control mode
        control_mode_all = (
            mut.PandaControlMode.kPosition
            | mut.PandaControlMode.kVelocity
            | mut.PandaControlMode.kTorque
        )
        command_rec_all = mut.PandaCommandReceiver(
            num_joints=mut.kPandaArmNumJoints, control_mode=control_mode_all
        )
        self.assertIsInstance(
            command_rec_all.get_commanded_velocity_output_port(), OutputPort
        )

    def test_franka_panda_command_sender(self):
        # Test with position and torque control mode
        control_mode = (
            mut.PandaControlMode.kPosition | mut.PandaControlMode.kTorque
        )
        command_send = mut.PandaCommandSender(
            num_joints=mut.kPandaArmNumJoints, control_mode=control_mode
        )
        self.assertIsInstance(command_send.get_position_input_port(), InputPort)
        self.assertIsInstance(command_send.get_torque_input_port(), InputPort)

        # Test with position, velocity, and torque control mode
        control_mode_all = (
            mut.PandaControlMode.kPosition
            | mut.PandaControlMode.kVelocity
            | mut.PandaControlMode.kTorque
        )
        command_send_all = mut.PandaCommandSender(
            num_joints=mut.kPandaArmNumJoints, control_mode=control_mode_all
        )
        self.assertIsInstance(
            command_send_all.get_velocity_input_port(), InputPort
        )

    def test_franka_panda_status_receiver(self):
        status_rec = mut.PandaStatusReceiver()
        self.assertIsInstance(
            status_rec.get_position_commanded_output_port(), OutputPort
        )
        self.assertIsInstance(
            status_rec.get_position_measured_output_port(), OutputPort
        )
        self.assertIsInstance(
            status_rec.get_velocity_commanded_output_port(), OutputPort
        )
        self.assertIsInstance(
            status_rec.get_velocity_measured_output_port(), OutputPort
        )
        self.assertIsInstance(
            status_rec.get_acceleration_commanded_output_port(), OutputPort
        )
        self.assertIsInstance(
            status_rec.get_torque_commanded_output_port(), OutputPort
        )
        self.assertIsInstance(
            status_rec.get_torque_measured_output_port(), OutputPort
        )
        self.assertIsInstance(
            status_rec.get_torque_external_output_port(), OutputPort
        )

        # Constructor variant.
        mut.PandaStatusReceiver(num_joints=mut.kPandaArmNumJoints)

    def test_franka_panda_status_sender(self):
        status_send = mut.PandaStatusSender()
        self.assertIsInstance(
            status_send.get_position_commanded_input_port(), InputPort
        )
        self.assertIsInstance(
            status_send.get_position_measured_input_port(), InputPort
        )
        self.assertIsInstance(
            status_send.get_velocity_commanded_input_port(), InputPort
        )
        self.assertIsInstance(
            status_send.get_velocity_measured_input_port(), InputPort
        )
        self.assertIsInstance(
            status_send.get_acceleration_commanded_input_port(), InputPort
        )
        self.assertIsInstance(
            status_send.get_torque_commanded_input_port(), InputPort
        )
        self.assertIsInstance(
            status_send.get_torque_measured_input_port(), InputPort
        )
        self.assertIsInstance(
            status_send.get_torque_external_input_port(), InputPort
        )

        # Constructor variant.
        mut.PandaStatusSender(num_joints=mut.kPandaArmNumJoints)
