# -*- coding: utf-8 -*-

import pydrake.multibody.inverse_kinematics as mut

import unittest
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.math import RigidTransform
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser


class TestPlanner(unittest.TestCase):
    def test_api(self):
        # Test existence for enumeration.
        enum = mut.DifferentialInverseKinematicsStatus
        enum.kSolutionFound
        enum.kNoSolutionFound
        enum.kStuck

        # Test results.
        results_cls = mut.DifferentialInverseKinematicsResult
        obj = results_cls(joint_velocities=[0, 1], status=enum.kSolutionFound)
        self.assertTrue((obj.joint_velocities == [0, 1]).all())
        self.assertEqual(obj.status, enum.kSolutionFound)

        # Test parameters.
        params_cls = mut.DifferentialInverseKinematicsParameters
        params = params_cls(num_positions=2, num_velocities=2)
        # Test existence.
        params.set_time_step(dt=0.2)
        self.assertEqual(params.get_time_step(), 0.2)
        params.get_num_positions
        params.get_nominal_joint_position
        params.get_num_velocities
        params.get_nominal_joint_position
        params.set_nominal_joint_position
        params.set_joint_centering_gain(K=np.eye(2))
        np.testing.assert_equal(params.get_joint_centering_gain(), np.eye(2))
        flag_E = [False, True, True, False, False, True]
        params.set_end_effector_velocity_flag(flag_E)
        np.testing.assert_equal(params.get_end_effector_velocity_flag(),
                                flag_E)
        params.get_joint_position_limits
        params.set_joint_position_limits
        params.get_joint_velocity_limits
        params.set_joint_velocity_limits
        params.get_joint_acceleration_limits
        params.set_joint_acceleration_limits
        params.set_maximum_scaling_to_report_stuck(scaling=0.1)
        self.assertEqual(params.get_maximum_scaling_to_report_stuck(), 0.1)
        params.set_end_effector_angular_speed_limit(speed=0.12)
        self.assertEqual(params.get_end_effector_angular_speed_limit(), 0.12)
        params.set_end_effector_translational_velocity_limits([-1, -2, -3],
                                                              [1, 2, 3])
        np.testing.assert_equal(
            params.get_end_effector_translational_velocity_limits()[0],
            [-1, -2, -3])
        np.testing.assert_equal(
            params.get_end_effector_translational_velocity_limits()[1],
            [1, 2, 3])

        # Test a basic call for the API. These values intentionally have no
        # physical meaning.
        result = mut.DoDifferentialInverseKinematics(
            q_current=[0, 1], v_current=[2, 3],
            V=[0, 1, 2, 3, 4, 5],
            J=np.array([
                [0, 1, 2, 3, 4, 5],
                [6, 7, 8, 9, 10, 11],
            ]).T,
            parameters=params)
        self.assertTrue(np.allclose(
            result.joint_velocities, [1, 0], atol=1e-8, rtol=0))
        self.assertEqual(result.status, enum.kSolutionFound)
        result = mut.DoDifferentialInverseKinematics(q_current=[0, 1],
                                                     v_current=[2, 3],
                                                     V=[0, 1, 2, 3, 4, 5],
                                                     J=np.array([
                                                         [0, 1, 2, 3, 4, 5],
                                                         [6, 7, 8, 9, 10, 11],
                                                     ]).T,
                                                     parameters=params,
                                                     N=np.eye(2),
                                                     Nplus=np.eye(2))
        self.assertTrue(np.allclose(
            result.joint_velocities, [1, 0], atol=1e-8, rtol=0))
        self.assertEqual(result.status, enum.kSolutionFound)

    def test_mbp_overloads(self):
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        plant = MultibodyPlant(0.0)
        Parser(plant).AddModels(file_name)
        plant.Finalize()

        context = plant.CreateDefaultContext()
        frame = plant.GetFrameByName("Link2")
        parameters = mut.DifferentialInverseKinematicsParameters(2, 2)

        mut.DoDifferentialInverseKinematics(plant, context,
                                            np.zeros(6), frame, parameters)

        mut.DoDifferentialInverseKinematics(plant, context, RigidTransform(),
                                            frame, parameters)

    def test_diff_ik_integrator(self):
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        plant = MultibodyPlant(0.0)
        Parser(plant).AddModels(file_name)
        plant.Finalize()

        context = plant.CreateDefaultContext()
        frame = plant.GetFrameByName("Link2")
        time_step = 0.1
        parameters = mut.DifferentialInverseKinematicsParameters(2, 2)

        integrator = mut.DifferentialInverseKinematicsIntegrator(
            robot=plant,
            frame_E=frame,
            time_step=time_step,
            parameters=parameters,
            robot_context=context,
            log_only_when_result_state_changes=True)

        integrator.get_mutable_parameters().set_time_step(0.2)
        self.assertEqual(integrator.get_parameters().get_time_step(), 0.2)
