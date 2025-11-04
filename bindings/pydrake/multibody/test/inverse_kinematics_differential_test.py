# -*- coding: utf-8 -*-

import pydrake.multibody.inverse_kinematics as mut  # ruff: isort: skip

import copy
import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.value import Value
from pydrake.common.yaml import yaml_dump_typed, yaml_load_typed
from pydrake.math import RigidTransform
from pydrake.multibody.math import SpatialVelocity
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import PrismaticJoint, SpatialInertia
from pydrake.planning import (
    CollisionCheckerParams,
    DofMask,
    JointLimits,
    RobotDiagramBuilder,
    SceneGraphCollisionChecker,
)
from pydrake.solvers import SolverId
from pydrake.systems.framework import BusValue


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
        np.testing.assert_equal(params.get_end_effector_velocity_flag(), flag_E)
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
        params.set_end_effector_translational_velocity_limits(
            [-1, -2, -3], [1, 2, 3]
        )
        np.testing.assert_equal(
            params.get_end_effector_translational_velocity_limits()[0],
            [-1, -2, -3],
        )
        np.testing.assert_equal(
            params.get_end_effector_translational_velocity_limits()[1],
            [1, 2, 3],
        )
        params.get_mutable_solver_options().SetOption(
            SolverId("dummy"), "dummy", 0.0
        )

        # Test a basic call for the API. These values intentionally have no
        # physical meaning.
        result = mut.DoDifferentialInverseKinematics(
            q_current=[0, 1],
            v_current=[2, 3],
            V=[0, 1, 2, 3, 4, 5],
            J=np.array(
                [
                    [0, 1, 2, 3, 4, 5],
                    [6, 7, 8, 9, 10, 11],
                ]
            ).T,
            parameters=params,
        )
        self.assertTrue(
            np.allclose(result.joint_velocities, [1, 0], atol=1e-8, rtol=0)
        )
        self.assertEqual(result.status, enum.kSolutionFound)
        result = mut.DoDifferentialInverseKinematics(
            q_current=[0, 1],
            v_current=[2, 3],
            V=[0, 1, 2, 3, 4, 5],
            J=np.array(
                [
                    [0, 1, 2, 3, 4, 5],
                    [6, 7, 8, 9, 10, 11],
                ]
            ).T,
            parameters=params,
            N=np.eye(2),
            Nplus=np.eye(2),
        )
        self.assertTrue(
            np.allclose(result.joint_velocities, [1, 0], atol=1e-8, rtol=0)
        )
        self.assertEqual(result.status, enum.kSolutionFound)

    def test_mbp_overloads(self):
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf"
        )
        plant = MultibodyPlant(0.0)
        Parser(plant).AddModels(file_name)
        plant.Finalize()

        context = plant.CreateDefaultContext()
        frame = plant.GetFrameByName("Link2")

        world_frame = plant.world_frame()

        parameters = mut.DifferentialInverseKinematicsParameters(2, 2)

        mut.DoDifferentialInverseKinematics(
            robot=plant,
            context=context,
            V_WE_desired=np.zeros(6),
            frame_E=frame,
            parameters=parameters,
        )

        mut.DoDifferentialInverseKinematics(
            robot=plant,
            context=context,
            X_WE_desired=RigidTransform(),
            frame_E=frame,
            parameters=parameters,
        )

        mut.DoDifferentialInverseKinematics(
            robot=plant,
            context=context,
            V_AE_desired=np.zeros(6),
            frame_A=world_frame,
            frame_E=frame,
            parameters=parameters,
        )

        mut.DoDifferentialInverseKinematics(
            robot=plant,
            context=context,
            X_AE_desired=RigidTransform(),
            frame_A=world_frame,
            frame_E=frame,
            parameters=parameters,
        )

    def test_diff_ik_integrator(self):
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf"
        )
        plant = MultibodyPlant(0.0)
        Parser(plant).AddModels(file_name)
        plant.Finalize()

        robot_context = plant.CreateDefaultContext()
        frame = plant.GetFrameByName("Link2")
        time_step = 0.1
        parameters = mut.DifferentialInverseKinematicsParameters(2, 2)

        integrator = mut.DifferentialInverseKinematicsIntegrator(
            robot=plant,
            frame_A=plant.world_frame(),
            frame_E=frame,
            time_step=time_step,
            parameters=parameters,
            robot_context=robot_context,
            log_only_when_result_state_changes=True,
        )

        context = integrator.CreateDefaultContext()
        X_AE = integrator.ForwardKinematics(context=context)

        integrator.get_mutable_parameters().set_time_step(0.2)
        self.assertEqual(integrator.get_parameters().get_time_step(), 0.2)

        integrator2 = mut.DifferentialInverseKinematicsIntegrator(
            robot=plant,
            frame_E=frame,
            time_step=time_step,
            parameters=parameters,
            robot_context=robot_context,
            log_only_when_result_state_changes=True,
        )

        context2 = integrator2.CreateDefaultContext()
        X_WE = integrator2.ForwardKinematics(context2)
        self.assertTrue(X_AE.IsExactlyEqualTo(X_WE))


class TestDiffIkSystems(unittest.TestCase):
    """Tests the DifferentialInverseKinematics framework classes (i.e.,
    DifferentialInverseKinematicsController and
    DifferentialInverseKinematicsSystem) and their attendant nested classes.
    """

    def test_recipe(self):
        Recipe = mut.DifferentialInverseKinematicsSystem.Recipe
        dut = Recipe()
        self.assertIsInstance(dut, Recipe)

        # Note: AddIngredient() is implicitly tested with each ingredient
        # (see _check_ingredient()).

    def _config_matches_params(self, config, params):
        """Tests that the attributes of config match the keys in params and,
        where possible (see below), the values likewise match.
        """
        for key, expected_value in params.items():
            dut_value = getattr(config, key)
            if isinstance(expected_value, dict):
                # If the attribute is a dictionary, we're assuming it is a
                # cartesian mask and explicitly test for it.
                for joint_name, dof_selector_expected in params[key].items():
                    self.assertIn(joint_name, dut_value)
                    np.testing.assert_array_equal(
                        dut_value[joint_name], dof_selector_expected
                    )
            elif isinstance(dut_value, np.ndarray):
                # params uses lists, but the config will probably contain
                # ndarray for the equivalent list; so test on dut_value type.
                np.testing.assert_array_equal(dut_value, expected_value)
            else:
                self.assertEqual(dut_value, expected_value)

    def _check_ingredient_config(self, dut_type, params):
        """Tests an ingredient Config struct's common APIs. This returns the
        instance of Config resulting from dut_type(**params).
        """
        # Confirm that each config has a default constructor as well.
        dut = dut_type()

        # But we'll use a Config with the specified parameters.
        dut = dut_type(**params)

        self.assertIsInstance(dut, dut_type)

        self._config_matches_params(dut, params)

        self.assertIn(list(params.keys())[0], repr(dut))
        copy.copy(dut)

        dumped = yaml_dump_typed(dut)
        from_yaml = yaml_load_typed(schema=dut_type, data=dumped)
        self._config_matches_params(from_yaml, params)

        return dut

    def _check_ingredient(self, dut_type, params1, params2, ctor=None):
        """Check the expected properties of an Ingredient. If the Ingredient
        is not simply constructed from an instance of its Config, then `ctor`
        should be provided to serve as the constructor.
        """
        config = self._check_ingredient_config(dut_type.Config, params1)

        dut = dut_type(config) if ctor is None else ctor(config)

        # Reading/writing config after the fact.
        self._config_matches_params(dut.GetConfig(), params1)

        dut.SetConfig(dut_type.Config(**params2))
        self._config_matches_params(dut.GetConfig(), params2)

        # Adds to a recipe.
        recipe = mut.DifferentialInverseKinematicsSystem.Recipe()
        recipe.AddIngredient(dut)

        return dut

    def test_least_squares_cost(self):
        Ingredient = mut.DifferentialInverseKinematicsSystem.LeastSquaresCost
        params = {
            "cartesian_qp_weight": 2,
            "cartesian_axis_masks": {"one": [1, 1, 1, 0, 0, 0]},
            "use_legacy_implementation": True,
        }

        self._check_ingredient(
            Ingredient, params, params | {"cartesian_qp_weight": 3}
        )

    def test_joint_centering_cost(self):
        Ingredient = mut.DifferentialInverseKinematicsSystem.JointCenteringCost
        params = {
            "posture_gain": 2,
            "cartesian_axis_masks": {"one": [1, 1, 1, 0, 0, 0]},
        }

        self._check_ingredient(Ingredient, params, params | {"posture_gain": 3})

    def test_cartesian_position_limit_constraint(self):
        DiffIk = mut.DifferentialInverseKinematicsSystem
        Ingredient = DiffIk.CartesianPositionLimitConstraint
        params = {"p_TG_next_lower": [-1, -2, -3], "p_TG_next_upper": [4, 5, 6]}

        self._check_ingredient(
            Ingredient, params, params | {"p_TG_next_lower": [-10, -20, -30]}
        )

    def test_cartesian_velocity_limit_constraint(self):
        DiffIk = mut.DifferentialInverseKinematicsSystem
        Ingredient = DiffIk.CartesianVelocityLimitConstraint
        params = {"V_next_TG_limit": [1, 2, 3, 4, 5, 6]}

        self._check_ingredient(
            Ingredient, params, {"V_next_TG_limit": [10, 20, 30, 40, 50, 60]}
        )

    def test_collision_constraint(self):
        DiffIk = mut.DifferentialInverseKinematicsSystem
        Ingredient = DiffIk.CollisionConstraint
        params = {"safety_distance": 1.5, "influence_distance": 2.5}

        dut = self._check_ingredient(
            Ingredient, params, params | {"safety_distance": 15}
        )

        # Note: this is an malformed selector function; but it is sufficient
        # to show that the corresponding setter has been bound. Evaluating the
        # functor is what would cause problems.
        def selector():
            pass

        dut.SetSelectDataForCollisionConstraintFunction(
            select_data_for_collision_constraint=selector
        )

    def test_joint_velocity_limit_constraint(self):
        DiffIk = mut.DifferentialInverseKinematicsSystem
        Ingredient = DiffIk.JointVelocityLimitConstraint
        params = {"min_margin": 1.5, "influence_margin": 2.5}

        # This constraint has a two-parameter constructor. So, we'll have to
        # provide one for the test harness. The value of the limits is
        # unimportant.
        limits = JointLimits(
            position_lower=[0, 0],
            position_upper=[1, 1],
            velocity_lower=[0, 0],
            velocity_upper=[1, 1],
            acceleration_lower=[0, 0],
            acceleration_upper=[1, 1],
        )

        def ctor(config):
            return Ingredient(config, limits)

        dut = self._check_ingredient(
            Ingredient, params, params | {"min_margin": 0.5}, ctor
        )

        np.testing.assert_array_equal(
            dut.GetJointLimits().position_lower(), [0, 0]
        )

        limits2 = JointLimits(
            position_lower=[-1, -1],
            position_upper=[1, 1],
            velocity_lower=[0, 0],
            velocity_upper=[1, 1],
            acceleration_lower=[0, 0],
            acceleration_upper=[1, 1],
        )
        dut.SetJointLimits(joint_limits=limits2)
        np.testing.assert_array_equal(
            dut.GetJointLimits().position_lower(), [-1, -1]
        )

    def _make_collision_checker(self):
        """This is a simplified version of the checker/robot diagram used in
        differential_inverse_kinematics_system_test.cc.
        """
        builder = RobotDiagramBuilder()
        plant = builder.plant()

        robot_index = plant.AddModelInstance("robot")
        body0 = plant.AddRigidBody("body0", robot_index, SpatialInertia.Zero())
        body1 = plant.AddRigidBody("ball", robot_index, SpatialInertia.Zero())
        plant.AddJoint(
            PrismaticJoint(
                "Wx", plant.world_frame(), body0.body_frame(), [1, 0, 0]
            )
        )
        plant.AddJoint(
            PrismaticJoint(
                "Wy", body0.body_frame(), body1.body_frame(), [0, 1, 0]
            )
        )
        plant.Finalize()

        robot = builder.Build()
        params = CollisionCheckerParams(
            model=robot,
            robot_model_instances=[robot_index],
            edge_step_size=0.1,
            env_collision_padding=0,
        )
        return SceneGraphCollisionChecker(params)

    def _make_diff_ik_system(self):
        """Makes a minimal DifferentialInverseKinematicsSystem."""
        DiffIk = mut.DifferentialInverseKinematicsSystem
        recipe = DiffIk.Recipe()
        recipe.AddIngredient(
            DiffIk.LeastSquaresCost(DiffIk.LeastSquaresCost.Config())
        )
        checker = self._make_collision_checker()
        active_dof = DofMask(2, True)  # 2 dofs in the plant.
        # No velocity limits on the two dofs of the robot.
        Vd_limit = SpatialVelocity([0, 0, 0], [np.inf, np.inf, 0])

        return DiffIk(
            recipe=recipe,
            task_frame="world",
            collision_checker=checker,
            active_dof=active_dof,
            time_step=0.1,
            K_VX=2,
            Vd_TG_limit=Vd_limit,
        )

    def test_diff_ik_system(self):
        """Builds a simple Diff IK system and evaluate it."""
        dut = self._make_diff_ik_system()

        # Note: we're going to need two copies of the context, but they share
        # some common values. This instance contains the common values.
        context_common = dut.CreateDefaultContext()
        q = [0, 0]
        dut.get_input_port_nominal_posture().FixValue(context_common, q)
        dut.get_input_port_position().FixValue(context_common, q)

        context_vel = copy.copy(context_common)
        desired_velocities = BusValue()
        desired_velocities.Set(
            "robot::ball", Value(SpatialVelocity([0, 0, 0], [1, 2, 0]))
        )
        dut.get_input_port_desired_cartesian_velocities().FixValue(
            context_vel, desired_velocities
        )

        command = dut.get_output_port_commanded_velocity().Eval(context_vel)

        np.testing.assert_allclose(command, [1, 2], atol=1e-4)

        # Now test stray APIs, untested in the above evaluation.
        self.assertIs(dut.plant(), dut.collision_checker().plant())
        self.assertEqual(dut.active_dof().count(), 2)
        self.assertEqual(dut.time_step(), 0.1)
        self.assertEqual(dut.task_frame().name(), "world")
        self.assertEqual(dut.K_VX(), 2)
        np.testing.assert_array_equal(
            dut.Vd_TG_limit().get_coeffs(), [0, 0, 0, np.inf, np.inf, 0]
        )
        self.assertIsInstance(
            dut.recipe(), mut.DifferentialInverseKinematicsSystem.Recipe
        )
        self.assertEqual(dut.recipe().num_ingredients(), 1)
        self.assertIsInstance(
            dut.recipe().ingredient(i=0),
            mut.DifferentialInverseKinematicsSystem.LeastSquaresCost,
        )

        # Note: setting poses in addition to velocities would cause an
        # evaluation error. We'll start with a fresh context (because we can't
        # unfix).
        context_pose = copy.copy(context_common)
        desired_poses = BusValue()
        desired_poses.Set("robot::ball", Value(RigidTransform([1, 2, 0])))
        dut.get_input_port_desired_cartesian_poses().FixValue(
            context_pose, desired_poses
        )

        command = dut.get_output_port_commanded_velocity().Eval(context_pose)
        # Current pose is at (0, 0), target pose is at (1, 2). The inferred
        # velocity is that displacement multiplied by K_VX / time_step = 20.
        expected_vel = [20, 40]
        np.testing.assert_allclose(command, expected_vel, atol=1e-3)

    def test_diff_ik_controller(self):
        Controller = mut.DifferentialInverseKinematicsController
        dut = Controller(
            differential_inverse_kinematics=self._make_diff_ik_system(),
            planar_rotation_dof_indices=[],
        )

        diff_ik_sys = dut.differential_inverse_kinematics()
        self.assertIsInstance(
            diff_ik_sys, mut.DifferentialInverseKinematicsSystem
        )
        self.assertIsInstance(
            dut.get_mutable_differential_inverse_kinematics(),
            mut.DifferentialInverseKinematicsSystem,
        )

        context = dut.CreateDefaultContext()

        positions = [0] * diff_ik_sys.plant().num_positions()
        dut.set_initial_position(context=context, value=positions)
