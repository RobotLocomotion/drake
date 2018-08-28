# -*- coding: utf-8 -*-

from pydrake.multibody.multibody_tree import (
    Body,
    BodyFrame,
    BodyIndex,
    ForceElement,
    ForceElementIndex,
    Frame,
    FrameIndex,
    Joint,
    JointActuator,
    JointActuatorIndex,
    JointIndex,
    ModelInstanceIndex,
    MultibodyTree,
    UniformGravityFieldElement,
    world_index,
)
from pydrake.multibody.multibody_tree.math import (
    SpatialVelocity,
)
from pydrake.multibody.multibody_tree.multibody_plant import (
    MultibodyPlant,
)
from pydrake.multibody.multibody_tree.parsing import (
    AddModelFromSdfFile,
)
from pydrake.multibody.benchmarks.acrobot import (
    AcrobotParameters,
    MakeAcrobotPlant,
)

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import (
    ConnectVisualization, DispatchLoadMessage, SceneGraph)
from pydrake.lcm import DrakeLcm
from pydrake.systems.framework import DiagramBuilder, InputPort, OutputPort
from pydrake.systems.analysis import Simulator

import copy
import unittest
import numpy as np


def get_index_class(cls):
    class_to_index_class_map = {
        Body: BodyIndex,
        ForceElement: ForceElementIndex,
        Joint: JointIndex,
        JointActuator: JointActuatorIndex,
    }
    for key_cls, index_cls in class_to_index_class_map.iteritems():
        if issubclass(cls, key_cls):
            return index_cls
    raise RuntimeError("Unknown class: {}".format(cls))


class TestMultibodyTreeMath(unittest.TestCase):
    def test_spatial_velocity(self):
        velocity = SpatialVelocity()
        # - Accessors.
        self.assertTrue(isinstance(velocity.rotational(), np.ndarray))
        self.assertTrue(isinstance(velocity.translational(), np.ndarray))
        self.assertEqual(velocity.rotational().shape, (3,))
        self.assertEqual(velocity.translational().shape, (3,))
        # - Fully-parameterized constructor.
        w = [0.1, 0.3, 0.5]
        v = [0., 1., 2.]
        velocity1 = SpatialVelocity(w=w, v=v)
        self.assertTrue(np.allclose(velocity1.rotational(), w))
        self.assertTrue(np.allclose(velocity1.translational(), v))


class TestMultibodyTree(unittest.TestCase):
    def test_type_safe_indices(self):
        self.assertEqual(world_index(), BodyIndex(0))

    def test_multibody_plant_api_via_parsing(self):
        # TODO(eric.cousineau): Decouple this when construction can be done
        # without parsing.
        # This a subset of `multibody_plant_sdf_parser_test.cc`.
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        plant = MultibodyPlant(time_step=0.01)
        model_instance = AddModelFromSdfFile(
            file_name=file_name, plant=plant, scene_graph=None)
        self.assertIsInstance(model_instance, ModelInstanceIndex)
        plant.Finalize()
        benchmark = MakeAcrobotPlant(AcrobotParameters(), True)
        self.assertEqual(plant.num_bodies(), benchmark.num_bodies())
        self.assertEqual(plant.num_joints(), benchmark.num_joints())
        self.assertEqual(plant.num_actuators(), benchmark.num_actuators())
        self.assertEqual(
            plant.num_model_instances(), benchmark.num_model_instances() + 1)
        self.assertEqual(plant.num_positions(), benchmark.num_positions())
        self.assertEqual(
            plant.num_positions(model_instance=model_instance),
            benchmark.num_positions())
        self.assertEqual(
            plant.num_velocities(), benchmark.num_velocities())
        self.assertEqual(
            plant.num_multibody_states(), benchmark.num_multibody_states())
        self.assertEqual(
            plant.num_actuated_dofs(), benchmark.num_actuated_dofs())
        self.assertTrue(plant.is_finalized())
        self.assertTrue(plant.HasBodyNamed(name="Link1"))
        self.assertTrue(plant.HasJointNamed(name="ShoulderJoint"))
        self._test_joint_api(plant.GetJointByName(name="ShoulderJoint"))
        self._test_joint_actuator_api(
            plant.GetJointActuatorByName(name="ElbowJoint"))
        self._test_body_api(plant.GetBodyByName(name="Link1"))
        self.assertIsInstance(
            plant.get_actuation_input_port(), InputPort)
        self.assertIsInstance(
            plant.get_continuous_state_output_port(), OutputPort)
        self.assertIsInstance(
            plant.get_contact_results_output_port(), OutputPort)

    def _test_multibody_tree_element_mixin(self, element):
        self.assertIsInstance(element.get_parent_tree(), MultibodyTree)
        cls = type(element)
        self.assertIsInstance(element.index(), get_index_class(cls))
        self.assertIsInstance(element.model_instance(), ModelInstanceIndex)

    def _test_body_api(self, body):
        self.assertIsInstance(body, Body)
        self._test_multibody_tree_element_mixin(body)
        self.assertIsInstance(body.name(), unicode)

    def _test_joint_api(self, joint):
        self.assertIsInstance(joint, Joint)
        self._test_multibody_tree_element_mixin(joint)
        self.assertIsInstance(joint.name(), unicode)
        self.assertIsInstance(joint.parent_body(), Body)
        self.assertIsInstance(joint.child_body(), Body)
        self.assertIsInstance(joint.frame_on_parent(), Frame)
        self.assertIsInstance(joint.frame_on_child(), Frame)
        self.assertIsInstance(joint.num_dofs(), int)

    def _test_joint_actuator_api(self, joint_actuator):
        self.assertIsInstance(joint_actuator, JointActuator)
        self._test_multibody_tree_element_mixin(joint_actuator)
        self.assertIsInstance(joint_actuator.name(), unicode)
        self.assertIsInstance(joint_actuator.joint(), Joint)

    def test_multibody_plant_parsing(self):
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")

        plant = MultibodyPlant(time_step=0.01)
        model_instance = AddModelFromSdfFile(
            file_name=file_name, plant=plant, scene_graph=None)
        self.assertIsInstance(model_instance, ModelInstanceIndex)

        plant = MultibodyPlant(time_step=0.01)
        model_instance = AddModelFromSdfFile(
            file_name=file_name, model_name="acrobot", plant=plant,
            scene_graph=None)

    def test_multibody_plant_simulation(self):
        """
        Provides an existence test by recreating
        `cart_pole_passive_simulation.cc` in Python.
        """
        file_name = FindResourceOrThrow(
            "drake/examples/multibody/cart_pole/cart_pole.sdf")
        builder = DiagramBuilder()
        scene_graph = builder.AddSystem(SceneGraph())
        cart_pole = builder.AddSystem(MultibodyPlant(time_step=0.))
        AddModelFromSdfFile(
            file_name=file_name, plant=cart_pole, scene_graph=scene_graph)
        cart_pole.AddForceElement(UniformGravityFieldElement([0, 0, -9.81]))
        cart_pole.Finalize(scene_graph)
        self.assertTrue(cart_pole.geometry_source_is_registered())

        builder.Connect(
            cart_pole.get_geometry_poses_output_port(),
            scene_graph.get_source_pose_port(cart_pole.get_source_id()))

        lcm = DrakeLcm()
        ConnectVisualization(scene_graph=scene_graph, builder=builder, lcm=lcm)
        diagram = builder.Build()
        DispatchLoadMessage(scene_graph=scene_graph, lcm=lcm)

        diagram_context = diagram.CreateDefaultContext()
        cart_pole_context = diagram.GetMutableSubsystemContext(
            cart_pole, diagram_context)

        cart_pole_context.FixInputPort(
            cart_pole.get_actuation_input_port().get_index(), [0])

        cart_slider = cart_pole.GetJointByName("CartSlider")
        pole_pin = cart_pole.GetJointByName("PolePin")
        cart_slider.set_translation(context=cart_pole_context, translation=0.)
        pole_pin.set_angle(context=cart_pole_context, angle=2.)

        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)
        simulator.Initialize()
        simulator.StepTo(0.1)
