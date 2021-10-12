# -*- coding: utf-8 -*-

import pydrake.manipulation.planner as mut

import unittest
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.math import RigidTransform
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.common.eigen_geometry import Isometry3


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
        params.get_timestep
        params.set_timestep
        params.get_num_positions
        params.get_nominal_joint_position
        params.get_num_velocities
        params.get_nominal_joint_position
        params.set_nominal_joint_position
        params.get_end_effector_velocity_gain
        params.set_end_effector_velocity_gain
        params.get_unconstrained_degrees_of_freedom_velocity_limit
        params.set_unconstrained_degrees_of_freedom_velocity_limit
        params.get_joint_position_limits
        params.set_joint_position_limits
        params.get_joint_velocity_limits
        params.set_joint_velocity_limits
        params.get_joint_acceleration_limits
        params.set_joint_acceleration_limits

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

    def test_mbp_overloads(self):
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        plant = MultibodyPlant(0.0)
        Parser(plant).AddModelFromFile(file_name)
        plant.Finalize()

        context = plant.CreateDefaultContext()
        frame = plant.GetFrameByName("Link2")
        parameters = mut.DifferentialInverseKinematicsParameters(2, 2)

        mut.DoDifferentialInverseKinematics(plant, context,
                                            np.zeros(6), frame, parameters)

        mut.DoDifferentialInverseKinematics(plant, context, RigidTransform(),
                                            frame, parameters)

        with catch_drake_warnings(expected_count=1):
            mut.DoDifferentialInverseKinematics(plant, context,
                                                Isometry3.Identity(), frame,
                                                parameters)

    def test_diff_ik_integrator(self):
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        plant = MultibodyPlant(0.0)
        Parser(plant).AddModelFromFile(file_name)
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

        integrator.get_mutable_parameters().set_timestep(0.2)
        self.assertEqual(integrator.get_parameters().get_timestep(), 0.2)
