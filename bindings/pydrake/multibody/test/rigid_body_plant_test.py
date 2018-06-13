import numpy as np
import os
import unittest

import pydrake.multibody.rigid_body_plant as mut

from pydrake.common import FindResourceOrThrow
from pydrake.lcm import DrakeMockLcm
from pydrake.systems.framework import BasicVector
from pydrake.multibody.rigid_body_tree import RigidBodyTree, FloatingBaseType


class TestRigidBodyPlant(unittest.TestCase):
    def _make_tree(self):
        urdf_path = FindResourceOrThrow(
            "drake/examples/pendulum/Pendulum.urdf")
        tree = RigidBodyTree(
                urdf_path, floating_base_type=FloatingBaseType.kFixed)
        return tree

    def test_api(self):
        for is_discrete in [False, True]:
            tree = self._make_tree()
            if is_discrete:
                timestep = 0.1
                plant = mut.RigidBodyPlant(tree, timestep)
            else:
                timestep = 0.0
                plant = mut.RigidBodyPlant(tree)
            model_id = 0

            # Tested in order in which they're declared in `multibody_py.cc`,
            # when able.
            plant.set_contact_model_parameters(
                mut.CompliantContactModelParameters())
            plant.set_default_compliant_material(mut.CompliantMaterial())
            self.assertTrue(plant.get_rigid_body_tree() is tree)
            self.assertEqual(plant.get_num_bodies(), tree.get_num_bodies())

            self.assertEqual(
                plant.get_num_positions(), tree.get_num_positions())
            self.assertEqual(
                plant.get_num_positions(model_id), tree.get_num_positions())

            self.assertEqual(
                plant.get_num_velocities(), tree.get_num_velocities())
            self.assertEqual(
                plant.get_num_velocities(model_id), tree.get_num_velocities())

            num_states = plant.get_num_positions() + plant.get_num_velocities()
            self.assertEqual(plant.get_num_states(), num_states)
            self.assertEqual(plant.get_num_states(model_id), num_states)

            num_actuators = 1
            self.assertEqual(plant.get_num_actuators(), num_actuators)
            self.assertEqual(plant.get_num_actuators(model_id), num_actuators)

            self.assertEqual(
                plant.get_num_model_instances(), 1)
            self.assertEqual(
                plant.get_input_size(), plant.get_num_actuators())
            self.assertEqual(plant.get_output_size(), plant.get_num_states())

            context = plant.CreateDefaultContext()
            x = plant.GetStateVector(context)
            plant.SetDefaultState(context, context.get_mutable_state())
            plant.set_position(context, 0, 1.)
            self.assertEqual(x[0], 1.)
            plant.set_velocity(context, 0, 2.)
            self.assertEqual(x[1], 2.)
            plant.set_state_vector(context, [3., 3.])
            self.assertTrue(np.allclose(x, [3., 3.]))
            plant.set_state_vector(context.get_mutable_state(), [4., 4.])
            self.assertTrue(np.allclose(x, [4., 4.]))

            self.assertEqual(
                plant.FindInstancePositionIndexFromWorldIndex(0, 0), 0)

            # Existence checks.
            self.assertTrue(plant.actuator_command_input_port() is not None)
            self.assertTrue(plant.model_instance_has_actuators(model_id))
            self.assertTrue(
                plant.model_instance_actuator_command_input_port(model_id)
                is not None)
            self.assertTrue(
                plant.state_output_port() is not None)
            self.assertTrue(
                plant.model_instance_state_output_port(model_id) is not None)
            self.assertTrue(
                plant.torque_output_port() is not None)
            self.assertTrue(
                plant.model_instance_torque_output_port(model_id) is not None)
            self.assertTrue(
                plant.kinematics_results_output_port() is not None)
            self.assertTrue(
                plant.contact_results_output_port() is not None)

            # Basic status.
            self.assertEqual(plant.is_state_discrete(), is_discrete)
            self.assertEqual(plant.get_time_step(), timestep)

    def test_contact_parameters_api(self):
        cls = mut.CompliantContactModelParameters
        param = cls()
        self.assertEqual(
            param.v_stiction_tolerance, cls.kDefaultVStictionTolerance)
        self.assertEqual(
            param.characteristic_radius, cls.kDefaultCharacteristicRadius)
        # Test simple mutuation.
        param.v_stiction_tolerance *= 2
        param.characteristic_radius *= 2
        # Test construction styles.
        param = cls(0.1, 0.2)
        param = cls(v_stiction_tolerance=0.1, characteristic_radius=0.2)

    def test_contact_result_api(self):
        # Test contact result bindings in isolation
        # by constructing dummy contact results
        results = mut.ContactResults()
        self.assertEqual(results.get_num_contacts(), 0)
        f = np.array([0., 1., 2.])
        results.set_generalized_contact_force(f)
        self.assertTrue(np.allclose(
            results.get_generalized_contact_force(), f))

        id_1 = 42
        id_2 = 43
        info = results.AddContact(element_a=id_1, element_b=id_2)
        self.assertEqual(results.get_num_contacts(), 1)
        info_dup = results.get_contact_info(0)
        self.assertIsInstance(info_dup, mut.ContactInfo)
        self.assertIs(info, info_dup)
        self.assertEqual(info.get_element_id_1(), id_1)
        self.assertEqual(info.get_element_id_2(), id_2)

        pt = np.array([0.1, 0.2, 0.3])
        normal = np.array([0.0, 1.0, 0.0])
        force = np.array([0.9, 0.1, 0.9])
        torque = np.array([0.6, 0.6, 0.6])
        cf = mut.ContactForce(application_point=pt,
                              normal=normal,
                              force=force,
                              torque=torque)
        self.assertTrue(np.allclose(cf.get_application_point(), pt))
        self.assertTrue(np.allclose(cf.get_force(), force))
        self.assertTrue(np.allclose(cf.get_normal_force(),
                                    normal*force.dot(normal)))
        self.assertTrue(np.allclose(cf.get_tangent_force(),
                                    force - normal*force.dot(normal)))
        self.assertTrue(np.allclose(cf.get_torque(), torque))
        self.assertTrue(np.allclose(cf.get_normal(), normal))

        results.Clear()
        self.assertEqual(results.get_num_contacts(), 0)

    def test_compliant_material_api(self):

        def flat_values(material):
            return (
                material.youngs_modulus(),
                material.dissipation(),
                material.static_friction(),
                material.dynamic_friction(),
            )

        cls = mut.CompliantMaterial
        material = cls()
        expected_default = (
            cls.kDefaultYoungsModulus,
            cls.kDefaultDissipation,
            cls.kDefaultStaticFriction,
            cls.kDefaultDynamicFriction,
        )
        self.assertEqual(flat_values(material), expected_default)
        # Test mutation by chaining.
        # Ensure these values and the defaults do not overlap.
        expected = tuple(2*x for x in expected_default)
        (material
            .set_youngs_modulus(expected[0])
            .set_dissipation(expected[1])
            .set_friction(expected[2], expected[3]))
        self.assertEqual(flat_values(material), expected)
        # Test mutation to default.
        material.set_youngs_modulus_to_default()
        self.assertTrue(material.youngs_modulus_is_default())
        material.set_dissipation_to_default()
        self.assertTrue(material.dissipation_is_default())
        material.set_friction_to_default()
        self.assertTrue(material.friction_is_default())
        # Test access with defaults.
        material_default = cls()
        self.assertEqual(
            material_default.youngs_modulus(default_value=expected[0]),
            expected[0])
        self.assertEqual(
            material_default.dissipation(default_value=expected[1]),
            expected[1])
        self.assertEqual(
            material_default.static_friction(default_value=expected[2]),
            expected[2])
        self.assertEqual(
            material_default.dynamic_friction(default_value=expected[3]),
            expected[3])
        # Test construction styles.
        material = cls(
            youngs_modulus=expected[0],
            dissipation=expected[1],
            static_friction=expected[2],
            dynamic_friction=expected[3])
        # Test basic errors.
        with self.assertRaises(RuntimeError):
            material.set_friction(0.3, 0.4)

    def test_drake_visualizer_api(self):
        tree = self._make_tree()
        lcm = DrakeMockLcm()
        # Test for existence.
        viz = mut.DrakeVisualizer(tree=tree, lcm=lcm, enable_playback=True)
        viz.set_publish_period(period=0.01)
        # - Force publish to ensure we have enough knot points.
        context = viz.CreateDefaultContext()
        x0 = np.zeros(tree.get_num_positions() + tree.get_num_velocities())
        context.FixInputPort(0, BasicVector(x0))
        context.set_time(0.)
        viz.Publish(context)
        # Use a small time period, since it uses realtime playback.
        context.set_time(0.01)
        viz.Publish(context)
        viz.ReplayCachedSimulation()
        # - Check that PublishLoadRobot can be called.
        viz.PublishLoadRobot()
