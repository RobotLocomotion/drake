# -*- coding: utf-8 -*-

import pydrake.manipulation.kuka_iiwa as mut

import unittest
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.lcm import DrakeLcm
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import (
    LoadModelDirectives,
    Parser,
    ProcessModelDirectives,
)
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import (
    DiagramBuilder,
    InputPort,
    OutputPort,
)
from pydrake.systems.lcm import LcmBuses


class TestKukaIiwa(unittest.TestCase):
    def test_constants(self):
        self.assertEqual(mut.kIiwaArmNumJoints, 7)
        self.assertIsInstance(mut.get_iiwa_max_joint_velocities(), np.ndarray)
        self.assertEqual(mut.kIiwaLcmStatusPeriod, 0.005)
        self.assertEqual(
            mut.IiwaControlMode.kDefault,
            mut.IiwaControlMode.kPosition | mut.IiwaControlMode.kTorque)

    def test_kuka_iiwa_lcm(self):
        command_rec = mut.IiwaCommandReceiver(
            num_joints=mut.kIiwaArmNumJoints,
            control_mode=mut.IiwaControlMode.kDefault)
        self.assertIsInstance(
            command_rec.get_message_input_port(), InputPort)
        self.assertIsInstance(
            command_rec.get_position_measured_input_port(), InputPort)
        self.assertIsInstance(
            command_rec.get_commanded_position_output_port(), OutputPort)
        self.assertIsInstance(
            command_rec.get_commanded_torque_output_port(), OutputPort)
        self.assertIsInstance(
            command_rec.get_time_output_port(), OutputPort)

        command_send = mut.IiwaCommandSender()
        self.assertIsInstance(
            command_send.get_time_input_port(), InputPort)
        self.assertIsInstance(
            command_send.get_position_input_port(), InputPort)
        self.assertIsInstance(
            command_send.get_torque_input_port(), InputPort)
        # Constructor variants.
        mut.IiwaCommandSender(
            num_joints=mut.kIiwaArmNumJoints,
            control_mode=mut.IiwaControlMode.kDefault)

        status_rec = mut.IiwaStatusReceiver()
        self.assertIsInstance(
            status_rec.get_time_measured_output_port(), OutputPort)
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
            status_send.get_time_measured_input_port(), InputPort)
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

    def test_kuka_iiwa_build_control(self):
        builder = DiagramBuilder()
        plant = builder.AddSystem(MultibodyPlant(1.))
        parser = Parser(plant)
        directives = LoadModelDirectives(FindResourceOrThrow(
            "drake/manipulation/util/test/iiwa7_wsg.dmd.yaml"))
        models_from_directives = ProcessModelDirectives(directives, parser)
        plant.Finalize()
        controller_plant = MultibodyPlant(1.)
        parser = Parser(controller_plant)
        parser.AddModels(FindResourceOrThrow(
            "drake/manipulation/models/iiwa_description/iiwa7/"
            "iiwa7_no_collision.sdf"))
        controller_plant.WeldFrames(
            controller_plant.world_frame(),
            controller_plant.GetFrameByName("iiwa_link_0"), RigidTransform())
        controller_plant.Finalize()

        self.assertEqual(len(builder.GetSystems()), 1)
        mut.BuildIiwaControl(
            plant=plant, iiwa_instance=plant.GetModelInstanceByName("iiwa7"),
            controller_plant=controller_plant, lcm=DrakeLcm(), builder=builder,
            ext_joint_filter_tau=0.12, desired_iiwa_kp_gains=np.arange(7),
            control_mode=mut.IiwaControlMode.kDefault)
        self.assertEqual(len(builder.GetSystems()), 12)

    def test_kuka_iiwa_driver(self):
        dut = mut.IiwaDriver()
        dut.hand_model_name = "schunk_wsg"
        dut.ext_joint_filter_tau = 0.12
        dut.lcm_bus = "test"
        self.assertIn("lcm_bus", repr(dut))

        builder = DiagramBuilder()
        plant = builder.AddSystem(MultibodyPlant(1.))
        parser = Parser(plant)
        directives = LoadModelDirectives(FindResourceOrThrow(
            "drake/manipulation/util/test/iiwa7_wsg.dmd.yaml"))
        models_from_directives = ProcessModelDirectives(directives, parser)
        plant.Finalize()
        model_dict = dict()
        for model in models_from_directives:
            model_dict[model.model_name] = model
        lcm_bus = LcmBuses()
        lcm_bus.Add("test", DrakeLcm())

        self.assertEqual(len(builder.GetSystems()), 1)
        mut.ApplyDriverConfig(
            driver_config=dut, model_instance_name="iiwa7",
            sim_plant=plant, models_from_directives=model_dict, lcms=lcm_bus,
            builder=builder)
        self.assertEqual(len(builder.GetSystems()), 13)
