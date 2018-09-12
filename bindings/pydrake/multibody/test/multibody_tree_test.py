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
    RevoluteJoint,
    UniformGravityFieldElement,
    WeldJoint,
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

import copy
import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.util.eigen_geometry import Isometry3
from pydrake.systems.framework import InputPort, OutputPort


def get_index_class(cls):
    # Maps a class to its corresponding index class, accommdating inheritance.
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
        self.assertIs(
            plant.GetBodyByName(name="Link1"),
            plant.GetBodyByName(name="Link1", model_instance=model_instance))
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
        self.assertIsInstance(joint.num_positions(), int)
        self.assertIsInstance(joint.num_velocities(), int)
        self.assertIsInstance(joint.position_start(), int)
        self.assertIsInstance(joint.velocity_start(), int)

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

    def test_multibody_tree_kinematics(self):
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        plant = MultibodyPlant()
        AddModelFromSdfFile(file_name, plant)
        plant.Finalize()
        context = plant.CreateDefaultContext()
        tree = plant.tree()
        world_frame = plant.world_frame()
        # TODO(eric.cousineau): Replace this with `GetFrameByName`.
        link1_frame = plant.GetBodyByName("Link1").body_frame()

        X_WL = tree.CalcRelativeTransform(
            context, frame_A=world_frame, frame_B=link1_frame)
        self.assertIsInstance(X_WL, Isometry3)

        p_AQi = tree.CalcPointsPositions(
            context=context, frame_B=link1_frame,
            p_BQi=np.array([[0, 1, 2], [10, 11, 12]]).T,
            frame_A=world_frame).T
        self.assertTupleEqual(p_AQi.shape, (2, 3))

        Jv_WL = tree.CalcFrameGeometricJacobianExpressedInWorld(
            context=context, frame_B=link1_frame,
            p_BoFo_B=[0, 0, 0])
        self.assertTupleEqual(Jv_WL.shape, (6, plant.num_velocities()))

    def test_multibody_state_access(self):
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        plant = MultibodyPlant()
        AddModelFromSdfFile(file_name, plant)
        plant.Finalize()
        context = plant.CreateDefaultContext()
        tree = plant.model()

        self.assertEqual(plant.num_positions(), 2)
        self.assertEqual(plant.num_velocities(), 2)

        q0 = np.array([3.14, 2.])
        v0 = np.array([-0.5, 1.])
        x0 = np.concatenate([q0, v0])

        # The default state is all values set to zero.
        x = tree.get_multibody_state_vector(context)
        self.assertTrue(np.allclose(x, np.zeros(4)))

        # Write into a mutable reference to the state vector.
        x_reff = tree.get_mutable_multibody_state_vector(context)
        x_reff[:] = x0

        # Verify we did modify the state stored in context.
        x = tree.get_multibody_state_vector(context)
        self.assertTrue(np.allclose(x, x0))

    def test_set_free_body_pose(self):
        file_name = FindResourceOrThrow(
            "drake/examples/double_pendulum/models/double_pendulum.sdf")
        plant = MultibodyPlant()
        plant_model = AddModelFromSdfFile(file_name, plant)
        plant.Finalize()

        context = plant.CreateDefaultContext()
        tree = plant.tree()
        X_WB_desired = Isometry3.Identity()
        R_WB = np.array([[0., 1., 0.],
                         [0., 0., 1.],
                         [1., 0., 0.]])
        X_WB_desired.set_rotation(R_WB)
        tree.SetFreeBodyPoseOrThrow(
            body=plant.GetBodyByName("base", plant_model),
            X_WB=X_WB_desired, context=context)

        world_frame = plant.world_frame()
        base_frame = plant.GetBodyByName("base").body_frame()

        X_WB = tree.CalcRelativeTransform(
            context, frame_A=world_frame, frame_B=base_frame)

        self.assertTrue(np.allclose(X_WB.matrix(), X_WB_desired.matrix()))

    def test_multibody_add_joint(self):
        """
        Tests joint constructors and `AddJoint`.
        """
        instance_file = FindResourceOrThrow(
            "drake/examples/double_pendulum/models/double_pendulum.sdf")
        # Add different joints between multiple model instances.
        # TODO(eric.cousineau): Remove the multiple instances and use
        # programmatically constructed bodies once this API is exposed in
        # Python.
        num_joints = 2
        plant = MultibodyPlant()
        instances = []
        for i in xrange(num_joints + 1):
            instance = AddModelFromSdfFile(
                instance_file, "instance_{}".format(i), plant)
            instances.append(instance)
        proximal_frame = "base"
        distal_frame = "lower_link"
        joints = [
            RevoluteJoint(
                name="revolve_things",
                frame_on_parent=plant.GetBodyByName(
                    distal_frame, instances[1]).body_frame(),
                frame_on_child=plant.GetBodyByName(
                    proximal_frame, instances[2]).body_frame(),
                axis=[0, 0, 1],
                damping=0.),
            WeldJoint(
                name="weld_things",
                parent_frame_P=plant.GetBodyByName(
                    distal_frame, instances[0]).body_frame(),
                child_frame_C=plant.GetBodyByName(
                    proximal_frame, instances[1]).body_frame(),
                X_PC=Isometry3.Identity()),
        ]
        for joint in joints:
            joint_out = plant.AddJoint(joint)
            self.assertIs(joint, joint_out)

        # The model is now complete.
        plant.Finalize()

        for joint in joints:
            self._test_joint_api(joint)
