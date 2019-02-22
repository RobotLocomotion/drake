from __future__ import print_function

# Module under test.
import pydrake.attic.systems.controllers as mut

import numpy as np
import unittest

from pydrake.common import FindResourceOrThrow
from pydrake.systems.framework import BasicVector
from pydrake.attic.multibody.rigid_body_tree import (
    FloatingBaseType, RigidBodyTree,
)


class TestControllers(unittest.TestCase):
    def test_inverse_dynamics(self):
        urdf_path = FindResourceOrThrow(
            "drake/manipulation/models/" +
            "iiwa_description/urdf/iiwa14_primitive_collision.urdf")
        tree = RigidBodyTree(urdf_path,
                             floating_base_type=FloatingBaseType.kFixed)
        num_v = tree.get_num_velocities()

        def compute_torque(tree, q, v, v_dot):
            cache = tree.doKinematics(q, v)
            return tree.massMatrix(cache).dot(v_dot) + \
                tree.dynamicsBiasTerm(cache, {})

        estimated_state_port = 0
        desired_acceleration_port = 1

        def check_torque_example(controller, q, v, v_dot_desired=None):
            if controller.is_pure_gravity_compensation():
                v_dot_desired = np.zeros(num_v)

            context = controller.CreateDefaultContext()

            x = np.concatenate([q, v])
            context.FixInputPort(estimated_state_port, BasicVector(x))
            if not controller.is_pure_gravity_compensation():
                context.FixInputPort(desired_acceleration_port,
                                     BasicVector(v_dot_desired))

            output = controller.AllocateOutput()
            controller.CalcOutput(context, output)
            expected_torque = compute_torque(tree, q, v, v_dot_desired)

            self.assertTrue(
                np.allclose(
                    output.get_vector_data(0).CopyToVector(), expected_torque))

        # Test with pure gravity compensation.
        controller = mut.RbtInverseDynamics(
            tree=tree,
            mode=mut.RbtInverseDynamics.InverseDynamicsMode.kGravityCompensation)  # noqa
        q = np.array([.1, .2, .3, .4, .5, .6, .7])
        v = np.zeros(num_v)
        check_torque_example(controller, q, v)

        # Test with desired acceleration.
        controller = mut.RbtInverseDynamics(
            tree=tree,
            mode=mut.RbtInverseDynamics.InverseDynamicsMode.kInverseDynamics)
        q = np.array([.7, .6, .5, .4, .3, .2, .1])
        v = np.array([-.1, -.2, -.3, -.4, -.5, -.6, -.7])
        v_dot_desired = np.array([-.1, .1, -.1, .1, -.1, .1, -.1])
        check_torque_example(controller, q, v, v_dot_desired)

    def test_inverse_dynamics_controller(self):
        urdf_path = FindResourceOrThrow(
            "drake/manipulation/models/" +
            "iiwa_description/urdf/iiwa14_primitive_collision.urdf")
        tree = RigidBodyTree(
                urdf_path, floating_base_type=FloatingBaseType.kFixed)
        kp = np.array([1., 2., 3., 4., 5., 6., 7.])
        ki = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
        kd = np.array([.5, 1., 1.5, 2., 2.5, 3., 3.5])

        controller = mut.RbtInverseDynamicsController(
            robot=tree,
            kp=kp,
            ki=ki,
            kd=kd,
            has_reference_acceleration=True)
        context = controller.CreateDefaultContext()
        output = controller.AllocateOutput()

        estimated_state_port = 0
        desired_state_port = 1
        desired_acceleration_port = 2
        control_port = 0

        self.assertEqual(
            controller.get_input_port(desired_acceleration_port).size(), 7)
        self.assertEqual(
            controller.get_input_port(estimated_state_port).size(), 14)
        self.assertEqual(
            controller.get_input_port(desired_state_port).size(), 14)
        self.assertEqual(
            controller.get_output_port(control_port).size(), 7)

        # current state
        q = np.array([-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3])
        v = np.array([-0.9, -0.6, -0.3, 0.0, 0.3, 0.6, 0.9])
        x = np.concatenate([q, v])

        # reference state and acceleration
        q_r = q + 0.1*np.ones_like(q)
        v_r = v + 0.1*np.ones_like(v)
        x_r = np.concatenate([q_r, v_r])
        vd_r = np.array([1., 2., 3., 4., 5., 6., 7.])

        integral_term = np.array([-1., -2., -3., -4., -5., -6., -7.])

        vd_d = vd_r + kp*(q_r-q) + kd*(v_r-v) + ki*integral_term

        context.FixInputPort(estimated_state_port, BasicVector(x))
        context.FixInputPort(desired_state_port, BasicVector(x_r))
        context.FixInputPort(desired_acceleration_port, BasicVector(vd_r))
        controller.set_integral_value(context, integral_term)

        # compute the expected torque
        cache = tree.doKinematics(q, v)
        expected_torque = tree.massMatrix(cache).dot(vd_d) + \
            tree.dynamicsBiasTerm(cache, {})

        controller.CalcOutput(context, output)
        self.assertTrue(np.allclose(output.get_vector_data(0).CopyToVector(),
                        expected_torque))
