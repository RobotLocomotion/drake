import numpy as np
import os
import unittest

from pydrake import getDrakePath
from pydrake.rbtree import RigidBodyTree, FloatingBaseType

import pydrake.multibody.rigid_body_plant as mut


class TestRigidBodyPlant(unittest.TestCase):
    def test_api(self):
        urdf_path = os.path.join(
            getDrakePath(), "examples/pendulum/Pendulum.urdf")
        for is_discrete in [False, True]:
            tree = RigidBodyTree(
                urdf_path, floating_base_type=FloatingBaseType.kFixed)
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
            self.assertEquals(plant.get_num_bodies(), tree.get_num_bodies())

            self.assertEquals(
                plant.get_num_positions(), tree.get_num_positions())
            self.assertEquals(
                plant.get_num_positions(model_id), tree.get_num_positions())

            self.assertEquals(
                plant.get_num_velocities(), tree.get_num_velocities())
            self.assertEquals(
                plant.get_num_velocities(model_id), tree.get_num_velocities())

            num_states = plant.get_num_positions() + plant.get_num_velocities()
            self.assertEquals(plant.get_num_states(), num_states)
            self.assertEquals(plant.get_num_states(model_id), num_states)

            num_actuators = 1
            self.assertEquals(plant.get_num_actuators(), num_actuators)
            self.assertEquals(plant.get_num_actuators(model_id), num_actuators)

            self.assertEquals(
                plant.get_num_model_instances(), 1)
            self.assertEquals(
                plant.get_input_size(), plant.get_num_actuators())
            self.assertEquals(plant.get_output_size(), plant.get_num_states())

            context = plant.CreateDefaultContext()
            x = plant.GetStateVector(context)
            plant.SetDefaultState(context, context.get_mutable_state())
            plant.set_position(context, 0, 1.)
            self.assertEquals(x[0], 1.)
            plant.set_velocity(context, 0, 2.)
            self.assertEquals(x[1], 2.)
            plant.set_state_vector(context, [3., 3.])
            self.assertTrue(np.allclose(x, [3., 3.]))
            plant.set_state_vector(context.get_mutable_state(), [4., 4.])
            self.assertTrue(np.allclose(x, [4., 4.]))

            self.assertEquals(
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
            self.assertEquals(plant.is_state_discrete(), is_discrete)
            self.assertEquals(plant.get_time_step(), timestep)

    def test_contact_parameters_api(self):
        cls = mut.CompliantContactModelParameters
        param = cls()
        self.assertEquals(
            param.v_stiction_tolerance, cls.kDefaultVStictionTolerance)
        self.assertEquals(
            param.characteristic_radius, cls.kDefaultCharacteristicRadius)
        # Test simple mutuation.
        param.v_stiction_tolerance *= 2
        param.characteristic_radius *= 2
        # Test construction styles.
        param = cls(0.1, 0.2)
        param = cls(v_stiction_tolerance=0.1, characteristic_radius=0.2)

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
        self.assertEquals(flat_values(material), expected_default)
        # Test mutation by chaining.
        # Ensure these values and the defaults do not overlap.
        expected = tuple(2*x for x in expected_default)
        (material
            .set_youngs_modulus(expected[0])
            .set_dissipation(expected[1])
            .set_friction(expected[2], expected[3]))
        self.assertEquals(flat_values(material), expected)
        # Test mutation to default.
        material.set_youngs_modulus_to_default()
        self.assertTrue(material.youngs_modulus_is_default())
        material.set_dissipation_to_default()
        self.assertTrue(material.dissipation_is_default())
        material.set_friction_to_default()
        self.assertTrue(material.friction_is_default())
        # Test access with defaults.
        material_default = cls()
        self.assertEquals(
            material_default.youngs_modulus(default_value=expected[0]),
            expected[0])
        self.assertEquals(
            material_default.dissipation(default_value=expected[1]),
            expected[1])
        self.assertEquals(
            material_default.static_friction(default_value=expected[2]),
            expected[2])
        self.assertEquals(
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


if __name__ == '__main__':
    unittest.main()
