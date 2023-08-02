# -*- coding: utf-8 -*-

import pydrake.manipulation as mut

import unittest
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.lcm import DrakeLcm
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
    VectorSystem,
)
from pydrake.systems.lcm import LcmBuses
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

        controller = mut.SchunkWsgController(kp=100., ki=2., kd=1.)
        self.assertIsInstance(
            controller.GetInputPort("command_message"), InputPort)
        self.assertIsInstance(controller.GetInputPort("state"), InputPort)
        self.assertIsInstance(controller.GetOutputPort("force"), OutputPort)

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
        self.assertIsInstance(
            mut.MakeMultibodyForceToWsgForceSystem(), VectorSystem)

    def test_schunk_wsg_build_control(self):
        builder = DiagramBuilder()
        plant = builder.AddSystem(MultibodyPlant(1.))
        parser = Parser(plant)
        directives = LoadModelDirectives(FindResourceOrThrow(
            "drake/manipulation/util/test/iiwa7_wsg.dmd.yaml"))
        models_from_directives = ProcessModelDirectives(directives, parser)
        plant.Finalize()

        self.assertEqual(len(builder.GetSystems()), 1)
        mut.BuildSchunkWsgControl(
            plant=plant,
            wsg_instance=plant.GetModelInstanceByName("schunk_wsg"),
            lcm=DrakeLcm(), builder=builder, pid_gains=[2., 3., 4.])
        self.assertEqual(len(builder.GetSystems()), 7)

    def test_schunk_wsg_driver(self):
        dut = mut.SchunkWsgDriver()
        dut.pid_gains = np.array([1., 2., 3.])
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
            driver_config=dut, model_instance_name="schunk_wsg",
            sim_plant=plant, models_from_directives=model_dict, lcms=lcm_bus,
            builder=builder)
        self.assertEqual(len(builder.GetSystems()), 7)
