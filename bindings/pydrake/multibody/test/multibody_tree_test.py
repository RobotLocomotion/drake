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
    ContactResults,
    MultibodyPlant,
    PointPairContactInfo,
)
from pydrake.multibody.multibody_tree.parsing import (
    AddModelFromSdfFile,
)
from pydrake.multibody.benchmarks.acrobot import (
    AcrobotParameters,
    MakeAcrobotPlant,
)

from pydrake.geometry import (
    GeometryId,
    PenetrationAsPointPair,
    SceneGraph,
)
from pydrake.systems.framework import DiagramBuilder


import copy
import math
import unittest

from six import text_type as unicode
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.util.eigen_geometry import Isometry3
from pydrake.systems.framework import InputPort, OutputPort
from pydrake.math import RollPitchYaw


def get_index_class(cls):
    # Maps a class to its corresponding index class, accommdating inheritance.
    class_to_index_class_map = {
        Body: BodyIndex,
        ForceElement: ForceElementIndex,
        Frame: FrameIndex,
        Joint: JointIndex,
        JointActuator: JointActuatorIndex,
    }
    for key_cls, index_cls in class_to_index_class_map.items():
        if issubclass(cls, key_cls):
            return index_cls
    raise RuntimeError("Unknown class: {}".format(cls))


def add_plant_and_scene_graph(builder):
    # TODO(eric.cousineau): Hoist to C++.
    plant = builder.AddSystem(MultibodyPlant())
    scene_graph = builder.AddSystem(SceneGraph())
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    builder.Connect(
        scene_graph.get_query_output_port(),
        plant.get_geometry_query_input_port())
    builder.Connect(
        plant.get_geometry_poses_output_port(),
        scene_graph.get_source_pose_port(plant.get_source_id()))
    return plant, scene_graph


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
            file_name=file_name, plant=plant)
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
        self.assertTrue(plant.HasBodyNamed(
            name="Link1", model_instance=model_instance))
        self.assertTrue(plant.HasJointNamed(name="ShoulderJoint"))
        self.assertTrue(plant.HasJointNamed(
            name="ShoulderJoint", model_instance=model_instance))
        shoulder = plant.GetJointByName(name="ShoulderJoint")
        self._test_joint_api(shoulder)
        np.testing.assert_array_equal(shoulder.lower_limits(), [-np.inf])
        np.testing.assert_array_equal(shoulder.upper_limits(), [np.inf])
        self.assertIs(shoulder, plant.GetJointByName(
            name="ShoulderJoint", model_instance=model_instance))
        self._test_joint_actuator_api(
            plant.GetJointActuatorByName(name="ElbowJoint"))
        self._test_body_api(plant.GetBodyByName(name="Link1"))
        self.assertIs(
            plant.GetBodyByName(name="Link1"),
            plant.GetBodyByName(name="Link1", model_instance=model_instance))
        self._test_frame_api(plant.GetFrameByName(name="Link1"))
        self.assertIs(
            plant.GetFrameByName(name="Link1"),
            plant.GetFrameByName(name="Link1", model_instance=model_instance))
        self.assertEqual(
            model_instance, plant.GetModelInstanceByName(name="acrobot"))
        self.assertIsInstance(
            plant.get_actuation_input_port(), InputPort)
        self.assertIsInstance(
            plant.get_continuous_state_output_port(), OutputPort)
        self.assertIsInstance(
            plant.get_contact_results_output_port(), OutputPort)
        tree = plant.tree()
        self.assertIsInstance(tree.num_frames(), int)
        self.assertIsInstance(tree.get_body(body_index=BodyIndex(0)), Body)
        self.assertIs(shoulder, tree.get_joint(joint_index=JointIndex(0)))
        self.assertIsInstance(tree.get_joint_actuator(
            actuator_index=JointActuatorIndex(0)), JointActuator)
        self.assertIsInstance(tree.get_frame(frame_index=FrameIndex(0)), Frame)
        self.assertEqual("acrobot", tree.GetModelInstanceName(
            model_instance=model_instance))

    def _test_multibody_tree_element_mixin(self, element):
        self.assertIsInstance(element.get_parent_tree(), MultibodyTree)
        cls = type(element)
        self.assertIsInstance(element.index(), get_index_class(cls))
        self.assertIsInstance(element.model_instance(), ModelInstanceIndex)

    def _test_frame_api(self, frame):
        self.assertIsInstance(frame, Frame)
        self._test_multibody_tree_element_mixin(frame)
        self.assertIsInstance(frame.name(), unicode)

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
            file_name=file_name, plant=plant)
        self.assertIsInstance(model_instance, ModelInstanceIndex)

        plant = MultibodyPlant(time_step=0.01)
        model_instance = AddModelFromSdfFile(
            file_name=file_name, model_name="acrobot", plant=plant)

    def test_multibody_tree_kinematics(self):
        file_name = FindResourceOrThrow(
            "drake/examples/double_pendulum/models/double_pendulum.sdf")
        plant = MultibodyPlant()
        AddModelFromSdfFile(file_name, plant)
        plant.Finalize()
        context = plant.CreateDefaultContext()
        tree = plant.tree()
        world_frame = plant.world_frame()
        base = plant.GetBodyByName("base")
        base_frame = plant.GetFrameByName("base")

        X_WL = tree.CalcRelativeTransform(
            context, frame_A=world_frame, frame_B=base_frame)
        self.assertIsInstance(X_WL, Isometry3)

        p_AQi = tree.CalcPointsPositions(
            context=context, frame_B=base_frame,
            p_BQi=np.array([[0, 1, 2], [10, 11, 12]]).T,
            frame_A=world_frame).T
        self.assertTupleEqual(p_AQi.shape, (2, 3))

        Jv_WL = tree.CalcFrameGeometricJacobianExpressedInWorld(
            context=context, frame_B=base_frame,
            p_BoFo_B=[0, 0, 0])
        self.assertTupleEqual(Jv_WL.shape, (6, plant.num_velocities()))

        # Compute body pose.
        X_WBase = tree.EvalBodyPoseInWorld(context, base)
        self.assertIsInstance(X_WBase, Isometry3)

        # All body poses.
        X_WB_list = tree.CalcAllBodyPosesInWorld(context)
        self.assertEqual(len(X_WB_list), 4)
        for X_WB in X_WB_list:
            self.assertIsInstance(X_WB, Isometry3)

        # Compute body velocities.
        v_WB_list = tree.CalcAllBodySpatialVelocitiesInWorld(context)
        self.assertEqual(len(v_WB_list), 4)
        for v_WB in v_WB_list:
            self.assertIsInstance(v_WB, SpatialVelocity)
        # - Sanity check.
        v_WBase_flat = np.hstack((
            v_WB_list[0].rotational(), v_WB_list[0].translational()))
        self.assertTrue(np.allclose(v_WBase_flat, np.zeros(6)))

        # Set pose for the base.
        X_WB_desired = Isometry3.Identity()
        X_WB = tree.CalcRelativeTransform(context, world_frame, base_frame)
        tree.SetFreeBodyPoseOrThrow(
            body=base, X_WB=X_WB_desired, context=context)
        self.assertTrue(np.allclose(X_WB.matrix(), X_WB_desired.matrix()))

        # Set a spatial velocity for the base.
        v_WB = SpatialVelocity(w=[1, 2, 3], v=[4, 5, 6])
        tree.SetFreeBodySpatialVelocityOrThrow(
            body=base, V_WB=v_WB, context=context)
        v_base = tree.EvalBodySpatialVelocityInWorld(context, base)
        self.assertTrue(np.allclose(v_base.rotational(), v_WB.rotational()))
        self.assertTrue(np.allclose(v_base.translational(),
                                    v_WB.translational()))

    def test_multibody_state_access(self):
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        plant = MultibodyPlant()
        AddModelFromSdfFile(file_name, plant)
        plant.Finalize()
        context = plant.CreateDefaultContext()
        tree = plant.tree()

        self.assertEqual(plant.num_positions(), 2)
        self.assertEqual(plant.num_velocities(), 2)

        q0 = np.array([3.14, 2.])
        v0 = np.array([-0.5, 1.])
        x0 = np.concatenate([q0, v0])

        # The default state is all values set to zero.
        x = tree.GetPositionsAndVelocities(context)
        self.assertTrue(np.allclose(x, np.zeros(4)))

        # Write into a mutable reference to the state vector.
        x_ref = tree.GetMutablePositionsAndVelocities(context)
        x_ref[:] = x0

        # Verify that positions and velocities were set correctly.
        self.assertTrue(np.allclose(plant.GetPositions(context), q0))
        self.assertTrue(np.allclose(plant.GetVelocities(context), v0))

        # Verify we did modify the state stored in context.
        x = tree.GetPositionsAndVelocities(context)
        self.assertTrue(np.allclose(x, x0))

        # Now set positions and velocities independently and check them.
        zeros_2 = np.zeros([2, 1])
        x_ref.fill(0)
        plant.SetPositions(context, q0)
        self.assertTrue(np.allclose(plant.GetPositions(context), q0))
        self.assertTrue(np.allclose(plant.GetVelocities(context), zeros_2))
        x_ref.fill(0)
        plant.SetVelocities(context, v0)
        self.assertTrue(np.allclose(plant.GetPositions(context), zeros_2))
        self.assertTrue(np.allclose(plant.GetVelocities(context), v0))

        # Now test SetPositionsAndVelocities().
        x_ref.fill(0)
        plant.SetPositionsAndVelocities(context, x0)
        self.assertTrue(np.allclose(
            tree.GetPositionsAndVelocities(context), x0))

    def test_model_instance_state_access(self):
        # Create a MultibodyPlant with a kuka arm and a schunk gripper.
        # the arm is welded to the world, the gripper is welded to the
        # arm's end effector.
        wsg50_sdf_path = FindResourceOrThrow(
            "drake/manipulation/models/" +
            "wsg_50_description/sdf/schunk_wsg_50.sdf")
        iiwa_sdf_path = FindResourceOrThrow(
            "drake/manipulation/models/" +
            "iiwa_description/sdf/iiwa14_no_collision.sdf")

        plant = MultibodyPlant()

        iiwa_model = AddModelFromSdfFile(
            file_name=iiwa_sdf_path, model_name='robot', plant=plant)
        gripper_model = AddModelFromSdfFile(
            file_name=wsg50_sdf_path, model_name='gripper', plant=plant)

        # Weld the base of arm and gripper to reduce the number of states.
        X_EeGripper = Isometry3.Identity()
        X_EeGripper.set_translation([0, 0, 0.081])
        X_EeGripper.set_rotation(
            RollPitchYaw(np.pi / 2, 0, np.pi / 2).ToRotationMatrix().matrix())
        plant.WeldFrames(A=plant.world_frame(),
                         B=plant.GetFrameByName("iiwa_link_0", iiwa_model))
        plant.WeldFrames(
            A=plant.GetFrameByName("iiwa_link_7", iiwa_model),
            B=plant.GetFrameByName("body", gripper_model),
            X_AB=X_EeGripper)
        plant.Finalize()

        # Create a context of the MBP and set the state of the context
        # to desired values.
        context = plant.CreateDefaultContext()
        tree = plant.tree()

        nq = plant.num_positions()
        nv = plant.num_velocities()

        nq_iiwa = 7
        nv_iiwa = 7
        nq_gripper = 2
        nv_gripper = 2
        q_iiwa_desired = np.zeros(nq_iiwa)
        v_iiwa_desired = np.zeros(nv_iiwa)
        q_gripper_desired = np.zeros(nq_gripper)
        v_gripper_desired = np.zeros(nv_gripper)

        q_iiwa_desired[2] = np.pi/3
        q_gripper_desired[0] = 0.1
        v_iiwa_desired[1] = 5.0
        q_gripper_desired[0] = -0.3

        x_iiwa_desired = np.zeros(nq_iiwa + nv_iiwa)
        x_iiwa_desired[0:nq_iiwa] = q_iiwa_desired
        x_iiwa_desired[nq_iiwa:nq_iiwa+nv_iiwa] = v_iiwa_desired

        x_gripper_desired = np.zeros(nq_gripper + nv_gripper)
        x_gripper_desired[0:nq_gripper] = q_gripper_desired
        x_gripper_desired[nq_gripper:nq_gripper+nv_gripper] = v_gripper_desired

        x_plant_desired = np.zeros(nq + nv)
        x_plant_desired[0:7] = q_iiwa_desired
        x_plant_desired[7:9] = q_gripper_desired
        x_plant_desired[nq:nq+7] = v_iiwa_desired
        x_plant_desired[nq+7:nq+nv] = v_gripper_desired

        # Check SetPositionsAndVelocities() for each model instance.
        # Do the iiwa model first.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        self.assertTrue(np.allclose(plant.GetPositionsAndVelocities(context),
                                    np.zeros(nq + nv)))
        plant.SetPositionsAndVelocities(context, iiwa_model, x_iiwa_desired)
        self.assertTrue(np.allclose(
            plant.GetPositionsAndVelocities(context, iiwa_model),
            x_iiwa_desired))
        self.assertTrue(np.allclose(plant.GetPositionsAndVelocities(
            context, gripper_model), np.zeros(nq_gripper + nv_gripper)))
        # Do the gripper model.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        self.assertTrue(np.allclose(plant.GetPositionsAndVelocities(context),
                                    np.zeros(nq + nv)))
        plant.SetPositionsAndVelocities(
            context, gripper_model, x_gripper_desired)
        self.assertTrue(np.allclose(
            plant.GetPositionsAndVelocities(context, gripper_model),
            x_gripper_desired))
        self.assertTrue(np.allclose(plant.GetPositionsAndVelocities(
            context, iiwa_model), np.zeros(nq_iiwa + nv_iiwa)))

        # Check SetPositions() for each model instance.
        # Do the iiwa model first.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        self.assertTrue(np.allclose(plant.GetPositionsAndVelocities(context),
                                    np.zeros(nq + nv)))
        plant.SetPositions(context, iiwa_model, q_iiwa_desired)
        self.assertTrue(np.allclose(
            plant.GetPositions(context, iiwa_model), q_iiwa_desired))
        self.assertTrue(np.allclose(plant.GetVelocities(
            context, iiwa_model), np.zeros(nv_iiwa)))
        self.assertTrue(np.allclose(plant.GetPositionsAndVelocities(
            context, gripper_model), np.zeros(nq_gripper + nv_gripper)))
        # Do the gripper model.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        self.assertTrue(np.allclose(plant.GetPositionsAndVelocities(context),
                                    np.zeros(nq + nv)))
        plant.SetPositions(context, gripper_model, q_gripper_desired)
        self.assertTrue(np.allclose(
            plant.GetPositions(context, gripper_model),
            q_gripper_desired))
        self.assertTrue(np.allclose(plant.GetVelocities(
            context, gripper_model), np.zeros(nq_gripper)))
        self.assertTrue(np.allclose(plant.GetPositionsAndVelocities(
            context, iiwa_model), np.zeros(nq_iiwa + nv_iiwa)))

        # Check SetVelocities() for each model instance.
        # Do the iiwa model first.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        self.assertTrue(np.allclose(plant.GetPositionsAndVelocities(context),
                                    np.zeros(nq + nv)))
        plant.SetVelocities(context, iiwa_model, v_iiwa_desired)
        self.assertTrue(np.allclose(
            plant.GetVelocities(context, iiwa_model), v_iiwa_desired))
        self.assertTrue(np.allclose(plant.GetPositions(
            context, iiwa_model), np.zeros(nq_iiwa)))
        self.assertTrue(np.allclose(plant.GetPositionsAndVelocities(
            context, gripper_model), np.zeros(nq_gripper + nv_gripper)))
        # Do the gripper model.
        plant.SetPositionsAndVelocities(context, np.zeros(nq + nv))
        self.assertTrue(np.allclose(plant.GetPositionsAndVelocities(context),
                                    np.zeros(nq + nv)))
        plant.SetVelocities(context, gripper_model, v_gripper_desired)
        self.assertTrue(np.allclose(
            plant.GetVelocities(context, gripper_model),
            v_gripper_desired))
        self.assertTrue(np.allclose(plant.GetPositions(
            context, gripper_model), np.zeros(nv_gripper)))
        self.assertTrue(np.allclose(plant.GetPositionsAndVelocities(
            context, iiwa_model), np.zeros(nq_iiwa + nv_iiwa)))

    def test_model_instance_state_access_by_array(self):
        # Create a MultibodyPlant with a kuka arm and a schunk gripper.
        # the arm is welded to the world, the gripper is welded to the
        # arm's end effector.
        wsg50_sdf_path = FindResourceOrThrow(
            "drake/manipulation/models/" +
            "wsg_50_description/sdf/schunk_wsg_50.sdf")
        iiwa_sdf_path = FindResourceOrThrow(
            "drake/manipulation/models/" +
            "iiwa_description/sdf/iiwa14_no_collision.sdf")

        timestep = 0.0002
        plant = MultibodyPlant(timestep)

        iiwa_model = AddModelFromSdfFile(
            file_name=iiwa_sdf_path, model_name='robot', plant=plant)
        gripper_model = AddModelFromSdfFile(
            file_name=wsg50_sdf_path, model_name='gripper', plant=plant)

        # Weld the base of arm and gripper to reduce the number of states.
        X_EeGripper = Isometry3.Identity()
        X_EeGripper.set_translation([0, 0, 0.081])
        X_EeGripper.set_rotation(
            RollPitchYaw(np.pi / 2, 0, np.pi / 2).
            ToRotationMatrix().matrix())
        plant.WeldFrames(
            A=plant.world_frame(),
            B=plant.GetFrameByName("iiwa_link_0", iiwa_model))
        plant.WeldFrames(
            A=plant.GetFrameByName("iiwa_link_7", iiwa_model),
            B=plant.GetFrameByName("body", gripper_model),
            X_AB=X_EeGripper)
        plant.Finalize()

        # Create a context of the MBP and set the state of the context
        # to desired values.
        context = plant.CreateDefaultContext()
        tree = plant.tree()

        nq = plant.num_positions()
        nv = plant.num_velocities()

        q_iiwa_desired = np.zeros(7)
        v_iiwa_desired = np.zeros(7)
        q_gripper_desired = np.zeros(2)
        v_gripper_desired = np.zeros(2)

        q_iiwa_desired[2] = np.pi/3
        q_gripper_desired[0] = 0.1
        v_iiwa_desired[1] = 5.0
        q_gripper_desired[0] = -0.3

        x_plant_desired = np.zeros(nq + nv)
        x_plant_desired[0:7] = q_iiwa_desired
        x_plant_desired[7:9] = q_gripper_desired
        x_plant_desired[nq:nq+7] = v_iiwa_desired
        x_plant_desired[nq+7:nq+nv] = v_gripper_desired

        x_plant = tree.GetMutablePositionsAndVelocities(context)
        x_plant[:] = x_plant_desired

        # Get state from context.
        x = tree.GetPositionsAndVelocities(context)
        q = x[0:nq]
        v = x[nq:nq+nv]

        # Get positions and velocities of specific model instances
        # from the postion/velocity vector of the plant.
        q_iiwa = tree.GetPositionsFromArray(iiwa_model, q)
        q_gripper = tree.GetPositionsFromArray(gripper_model, q)
        v_iiwa = tree.GetVelocitiesFromArray(iiwa_model, v)
        v_gripper = tree.GetVelocitiesFromArray(gripper_model, v)

        # Assert that the GetPositionsFromArray return
        # the desired values set earlier.
        self.assertTrue(np.allclose(q_iiwa_desired, q_iiwa))
        self.assertTrue(np.allclose(v_iiwa_desired, v_iiwa))
        self.assertTrue(np.allclose(q_gripper_desired, q_gripper))
        self.assertTrue(np.allclose(v_gripper_desired, v_gripper))

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
        for i in range(num_joints + 1):
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

    def test_multibody_dynamics(self):
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        plant = MultibodyPlant()
        AddModelFromSdfFile(file_name, plant)
        plant.Finalize()
        context = plant.CreateDefaultContext()
        tree = plant.tree()

        H = tree.CalcMassMatrixViaInverseDynamics(context)
        Cv = tree.CalcBiasTerm(context)

        self.assertTrue(H.shape == (2, 2))
        self.assertTrue(Cv.shape == (2, ))

    def test_contact(self):
        # PenetrationAsContactPair
        point_pair = PenetrationAsPointPair()
        self.assertTrue(isinstance(point_pair.id_A, GeometryId))
        self.assertTrue(isinstance(point_pair.id_B, GeometryId))
        self.assertTrue(point_pair.p_WCa.shape == (3,))
        self.assertTrue(point_pair.p_WCb.shape == (3,))
        self.assertTrue(isinstance(point_pair.depth, float))

        # PointPairContactInfo
        id_A = BodyIndex(0)
        id_B = BodyIndex(1)
        contact_info = PointPairContactInfo(
            bodyA_index=id_A, bodyB_index=id_B,
            f_Bc_W=np.array([0, 0, 1]), p_WC=np.array([0, 0, 0]),
            separation_speed=0, slip_speed=0, point_pair=point_pair)
        self.assertTrue(
            isinstance(contact_info.bodyA_index(), BodyIndex))
        self.assertTrue(
            isinstance(contact_info.bodyB_index(), BodyIndex))
        self.assertTrue(contact_info.contact_force().shape == (3,))
        self.assertTrue(contact_info.contact_point().shape == (3,))
        self.assertTrue(isinstance(contact_info.slip_speed(), float))

        # ContactResults
        contact_results = ContactResults()
        contact_results.AddContactInfo(contact_info)
        self.assertTrue(contact_results.num_contacts() == 1)
        self.assertTrue(
            isinstance(contact_results.contact_info(0), PointPairContactInfo))

    def test_scene_graph_queries(self):
        builder = DiagramBuilder()
        plant, scene_graph = add_plant_and_scene_graph(builder)
        AddModelFromSdfFile(
            FindResourceOrThrow(
                "drake/bindings/pydrake/multibody/test/two_bodies.sdf"),
            plant, scene_graph)
        plant.Finalize(scene_graph)
        diagram = builder.Build()
        # The model `two_bodies` has two (implicitly) floating bodies that are
        # placed in the same position. The default state would be for these two
        # bodies to be coincident, and thus collide.
        context = diagram.CreateDefaultContext()
        sg_context = diagram.GetMutableSubsystemContext(scene_graph, context)
        query_object = scene_graph.get_query_output_port().Eval(sg_context)
        # Implicitly require that this should be size 1.
        point_pair, = query_object.ComputePointPairPenetration()
        self.assertIsInstance(point_pair, PenetrationAsPointPair)
        inspector = query_object.inspector()
        bodies = {plant.GetBodyFromFrameId(inspector.GetFrameId(id_))
                  for id_ in [point_pair.id_A, point_pair.id_B]}
        self.assertSetEqual(
            bodies,
            {plant.GetBodyByName("body1"), plant.GetBodyByName("body2")})
