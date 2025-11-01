# -*- coding: utf-8 -*-

import pydrake.manipulation as mut  # ruff: isort: skip

import gc
import unittest
import weakref

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
from pydrake.systems.test.test_util import call_build_from_cpp


class TestKukaIiwa(unittest.TestCase):
    def make_builder_plant_controller_plant(self):
        builder = DiagramBuilder()
        plant = builder.AddSystem(MultibodyPlant(1.0))
        parser = Parser(plant)
        directives = LoadModelDirectives(
            FindResourceOrThrow(
                "drake/manipulation/util/test/iiwa7_wsg.dmd.yaml"
            )
        )
        ProcessModelDirectives(directives, parser)
        plant.Finalize()
        controller_plant = MultibodyPlant(1.0)
        parser = Parser(controller_plant)
        parser.AddModels(
            url=(
                "package://drake_models/iiwa_description/sdf/"
                + "iiwa7_no_collision.sdf"
            )
        )
        controller_plant.WeldFrames(
            controller_plant.world_frame(),
            controller_plant.GetFrameByName("iiwa_link_0"),
            RigidTransform(),
        )
        controller_plant.Finalize()
        return builder, plant, controller_plant

    def test_constants(self):
        self.assertEqual(mut.kIiwaArmNumJoints, 7)
        self.assertIsInstance(mut.get_iiwa_max_joint_velocities(), np.ndarray)
        self.assertEqual(mut.kIiwaLcmStatusPeriod, 0.005)
        self.assertIsInstance(
            mut.IiwaControlMode.kPositionOnly, mut.IiwaControlMode
        )
        self.assertIsInstance(
            mut.IiwaControlMode.kTorqueOnly, mut.IiwaControlMode
        )
        self.assertIsInstance(
            mut.IiwaControlMode.kPositionAndTorque, mut.IiwaControlMode
        )
        control_mode = mut.IiwaControlMode.kPositionAndTorque
        self.assertTrue(mut.position_enabled(control_mode=control_mode))
        self.assertTrue(mut.torque_enabled(control_mode=control_mode))
        self.assertEqual(
            mut.ParseIiwaControlMode(control_mode="position_and_torque"),
            control_mode,
        )

    def test_kuka_iiwa_lcm(self):
        command_rec = mut.IiwaCommandReceiver(
            num_joints=mut.kIiwaArmNumJoints,
            control_mode=mut.IiwaControlMode.kPositionAndTorque,
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
        self.assertIsInstance(command_rec.get_time_output_port(), OutputPort)

        command_send = mut.IiwaCommandSender()
        self.assertIsInstance(command_send.get_time_input_port(), InputPort)
        self.assertIsInstance(command_send.get_position_input_port(), InputPort)
        self.assertIsInstance(command_send.get_torque_input_port(), InputPort)
        # Constructor variants.
        mut.IiwaCommandSender(
            num_joints=mut.kIiwaArmNumJoints,
            control_mode=mut.IiwaControlMode.kPositionAndTorque,
        )

        status_rec = mut.IiwaStatusReceiver()
        self.assertIsInstance(
            status_rec.get_time_measured_output_port(), OutputPort
        )
        self.assertIsInstance(
            status_rec.get_position_commanded_output_port(), OutputPort
        )
        self.assertIsInstance(
            status_rec.get_position_measured_output_port(), OutputPort
        )
        self.assertIsInstance(
            status_rec.get_velocity_estimated_output_port(), OutputPort
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

        status_send = mut.IiwaStatusSender()
        self.assertIsInstance(
            status_send.get_time_measured_input_port(), InputPort
        )
        self.assertIsInstance(
            status_send.get_position_commanded_input_port(), InputPort
        )
        self.assertIsInstance(
            status_send.get_position_measured_input_port(), InputPort
        )
        self.assertIsInstance(
            status_send.get_velocity_estimated_input_port(), InputPort
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

    def test_kuka_iiwa_api(self):
        self.assertEqual(mut.get_iiwa_max_joint_velocities().shape, (7,))

    def test_kuka_iiwa_build_control(self):
        builder, plant, controller_plant = (
            self.make_builder_plant_controller_plant()
        )
        tare = len(builder.GetSystems())
        mut.BuildIiwaControl(
            builder=builder,
            lcm=DrakeLcm(),
            plant=plant,
            iiwa_instance=plant.GetModelInstanceByName("iiwa7"),
            driver_config=mut.IiwaDriver(),
            controller_plant=controller_plant,
        )
        self.assertGreater(len(builder.GetSystems()), tare)

    def test_kuka_iiwa_driver(self):
        dut = mut.IiwaDriver()
        dut.hand_model_name = "schunk_wsg"
        dut.ext_joint_filter_tau = 0.12
        dut.lcm_bus = "test"
        self.assertIn("lcm_bus", repr(dut))

        builder = DiagramBuilder()
        plant = builder.AddSystem(MultibodyPlant(1.0))
        parser = Parser(plant)
        directives = LoadModelDirectives(
            FindResourceOrThrow(
                "drake/manipulation/util/test/iiwa7_wsg.dmd.yaml"
            )
        )
        models_from_directives = ProcessModelDirectives(directives, parser)
        plant.Finalize()
        model_dict = dict()
        for model in models_from_directives:
            model_dict[model.model_name] = model
        lcm_bus = LcmBuses()
        lcm_bus.Add("test", DrakeLcm())

        tare = len(builder.GetSystems())
        mut.ApplyDriverConfig(
            driver_config=dut,
            model_instance_name="iiwa7",
            sim_plant=plant,
            models_from_directives=model_dict,
            lcms=lcm_bus,
            builder=builder,
        )
        self.assertGreater(len(builder.GetSystems()), tare)

    def test_kuka_iiwa_sim_driver(self):
        builder, plant, controller_plant = (
            self.make_builder_plant_controller_plant()
        )

        dut = mut.SimIiwaDriver(
            driver_config=mut.IiwaDriver(),
            controller_plant=controller_plant,
        )
        self.assertGreater(dut.num_input_ports(), 0)
        self.assertGreater(dut.num_output_ports(), 0)

        tare = len(builder.GetSystems())
        dut1 = mut.SimIiwaDriver.AddToBuilder(
            builder=builder,
            plant=plant,
            iiwa_instance=plant.GetModelInstanceByName("iiwa7"),
            driver_config=mut.IiwaDriver(),
            controller_plant=controller_plant,
        )
        self.assertGreater(dut1.num_input_ports(), 0)
        self.assertGreater(dut1.num_output_ports(), 0)
        self.assertGreater(len(builder.GetSystems()), tare)

    def test_kuka_iiwa_sim_driver_lifetime_init(self):
        def make_diagram():
            builder, plant, controller_plant = (
                self.make_builder_plant_controller_plant()
            )
            # `dut` is a diagram; its init() promises to keep
            # `controller_plant` alive.
            dut = mut.SimIiwaDriver(
                driver_config=mut.IiwaDriver(),
                controller_plant=controller_plant,
            )
            return dut

        diagram = make_diagram()
        gc.collect()
        # Crashes if controller_plant is not kept alive by bindings.
        ad_diagram = diagram.ToAutoDiffXd()  # noqa: F841 (unused-variable)

    def call_build_from(self, diagram_builder, language):
        assert language in ["python", "c++"]
        if language == "python":
            # The pydrake binding ensures object lifetimes properly.
            return diagram_builder.Build()
        else:
            # Calling build on a python-populated diagram builder from c++ is
            # dodgy at best, but don't worry -- all will be well if something
            # retains ownership of the diagram.
            return call_build_from_cpp(diagram_builder)

    def test_kuka_iiwa_sim_driver_lifetime_add_to_builder(self):
        def make_diagram(call_build_from_language):
            builder, plant, controller_plant = (
                self.make_builder_plant_controller_plant()
            )
            dut = mut.SimIiwaDriver.AddToBuilder(
                builder=builder,
                plant=plant,
                iiwa_instance=plant.GetModelInstanceByName("iiwa7"),
                driver_config=mut.IiwaDriver(),
                controller_plant=controller_plant,
            )
            diagram = self.call_build_from(builder, call_build_from_language)
            return diagram, dut

        for call_build_from_language in ["python", "c++"]:
            # In either case here, the diagram reference is sufficient to keep
            # the whole diagram and its dependencies alive.
            diagram, dut = make_diagram(call_build_from_language)
            del dut
            gc.collect()
            # Crashes if controller_plant is not kept alive by bindings.
            ad_diagram = diagram.ToAutoDiffXd()  # noqa: F841 (unused-variable)
            # The diagram is mortal.
            spy = weakref.finalize(diagram, lambda: None)
            del diagram
            gc.collect()
            self.assertFalse(spy.alive)

        # In a python-only program, any subsystem should be sufficient to keep
        # everything alive.
        diagram, dut = make_diagram("python")
        del diagram
        gc.collect()
        # Crashes if controller_plant is not kept alive by bindings.
        ad_dut = dut.ToAutoDiffXd()  # noqa: F841 (unused-variable)
        # The dut is mortal.
        spy = weakref.finalize(dut, lambda: None)
        del dut
        gc.collect()
        self.assertFalse(spy.alive)

    def test_kuka_iiwa_sim_driver_lifetime_build_iiwa_control(self):
        def make_diagram(oblivious=False):
            builder, plant, controller_plant = (
                self.make_builder_plant_controller_plant()
            )
            mut.BuildIiwaControl(
                builder=builder,
                lcm=DrakeLcm(),
                plant=plant,
                iiwa_instance=plant.GetModelInstanceByName("iiwa7"),
                driver_config=mut.IiwaDriver(),
                controller_plant=controller_plant,
            )
            if oblivious:
                diagram = call_build_from_cpp(builder)
            else:
                diagram = builder.Build()
            return diagram, weakref.finalize(controller_plant, lambda: None)

        for call_build_from_language in ["python", "c++"]:
            # In either case here, the diagram reference is sufficient to keep
            # the whole diagram and its dependencies alive. In this case, we
            # care about the lifetime of controller_plant, but we can't use
            # scalar conversion to flush it out. Use the returned weakref spy
            # instead.
            diagram, spy = make_diagram(call_build_from_language)
            gc.collect()
            self.assertTrue(spy.alive, (spy.alive, call_build_from_language))
            # The diagram and its dependencies are mortal.
            spy = weakref.finalize(diagram, lambda: None)
            del diagram
            gc.collect()
            self.assertFalse(spy.alive)
