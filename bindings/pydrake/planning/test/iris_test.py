import unittest

import pydrake.planning as mut
from pydrake.common import RandomGenerator, Parallelism
from pydrake.geometry.optimization import Hyperellipsoid, HPolyhedron
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.multibody.rational import RationalForwardKinematics
from pydrake.planning import (
    RobotDiagramBuilder,
    SceneGraphCollisionChecker,
    CollisionCheckerParams,
    IrisParameterizationFunction,
)
from pydrake.symbolic import Variable

import numpy as np


class TestIrisZo(unittest.TestCase):
    def test_iris_zo(self):
        # Taken from iris_from_clique_cover_test.py
        cross_cspace_urdf = """
        <robot name="boxes">
          <link name="fixed">
            <collision name="top_left">
              <origin rpy="0 0 0" xyz="-1 1 0"/>
              <geometry><box size="1 1 1"/></geometry>
            </collision>
            <collision name="top_right">
              <origin rpy="0 0 0" xyz="1 1 0"/>
              <geometry><box size="1 1 1"/></geometry>
            </collision>
            <collision name="bottom_left">
              <origin rpy="0 0 0" xyz="-1 -1 0"/>
              <geometry><box size="1 1 1"/></geometry>
            </collision>
            <collision name="bottom_right">
              <origin rpy="0 0 0" xyz="1 -1 0"/>
              <geometry><box size="1 1 1"/></geometry>
            </collision>
          </link>
          <joint name="fixed_link_weld" type="fixed">
            <parent link="world"/>
            <child link="fixed"/>
          </joint>
          <link name="movable">
            <collision name="sphere">
              <geometry><sphere radius="0.1"/></geometry>
            </collision>
          </link>
          <link name="for_joint"/>
          <joint name="x" type="prismatic">
            <axis xyz="1 0 0"/>
            <limit lower="-2" upper="2"/>
            <parent link="world"/>
            <child link="for_joint"/>
          </joint>
          <joint name="y" type="prismatic">
            <axis xyz="0 1 0"/>
            <limit lower="-2" upper="2"/>
            <parent link="for_joint"/>
            <child link="movable"/>
          </joint>
        </robot>
"""
        params = dict(edge_step_size=0.1)
        builder = RobotDiagramBuilder()
        params["robot_model_instances"] = builder.parser().AddModelsFromString(
            cross_cspace_urdf, "urdf"
        )
        params["model"] = builder.Build()
        plant = params["model"].plant()
        checker = SceneGraphCollisionChecker(**params)
        seed_point = np.zeros((2,))
        options = mut.IrisZoOptions()
        options.sampled_iris_options.num_particles = 1000
        options.sampled_iris_options.tau = 0.5
        options.sampled_iris_options.delta = 5e-2
        options.sampled_iris_options.epsilon = 1e-2
        options.sampled_iris_options.\
            containment_points = np.array([[0, 0], [1, 0]])
        options.sampled_iris_options.max_iterations = 3
        options.sampled_iris_options.max_iterations_separating_planes = 20
        options.sampled_iris_options.max_separating_planes_per_iteration = -1
        options.bisection_steps = 10
        options.sampled_iris_options.parallelism = Parallelism(True)
        options.sampled_iris_options.verbose = False
        options.sampled_iris_options.configuration_space_margin = 1e-2
        options.sampled_iris_options.termination_threshold = 1e-2
        options.sampled_iris_options.relative_termination_threshold = 1e-3
        options.sampled_iris_options.random_seed = 1337
        options.sampled_iris_options.mixing_steps = 50
        starting_ellipsoid = Hyperellipsoid.MakeHypersphere(0.01, seed_point)
        options.sampled_iris_options.\
            prog_with_additional_constraints = InverseKinematics(
                plant
            ).prog()
        domain = HPolyhedron.MakeBox(plant.GetPositionLowerLimits(),
                                     plant.GetPositionUpperLimits())
        region = mut.IrisZo(checker=checker,
                            starting_ellipsoid=starting_ellipsoid,
                            domain=domain,
                            options=options)
        test_point = np.array([0.0, 0.5])
        self.assertTrue(region.PointInSet(test_point))
        test_point2 = np.array([0.0, 1])
        self.assertTrue(region.PointInSet(test_point2))

        kin = RationalForwardKinematics(plant)
        q_star = np.zeros(2)
        options.parameterization = mut.IrisParameterizationFunction(
            kin=kin,
            q_star_val=q_star)
        self.assertTrue(
            options.parameterization.get_parameterization_is_threadsafe())
        self.assertEqual(
            options.parameterization.get_parameterization_dimension(), 2)
        self.assertTrue(
            callable(options.parameterization.get_parameterization()))
        s = np.array([0, 1])
        q = options.parameterization.get_parameterization()(s)
        self.assertTrue(np.allclose(q,
                                    kin.ComputeQValue(s, q_star), atol=0))

        options2 = mut.IrisZoOptions()
        options2.parameterization = IrisParameterizationFunction(
            options.parameterization.get_parameterization(),
            options.parameterization.get_parameterization_dimension())
        self.assertFalse(
            options2.parameterization.get_parameterization_is_threadsafe())
        self.assertEqual(
            options2.parameterization.get_parameterization_dimension(), 2)
        self.assertTrue(
            callable(options2.parameterization.get_parameterization()))
        q2 = options2.parameterization.get_parameterization()(np.array(s))
        self.assertTrue(np.allclose(q2,
                                    kin.ComputeQValue(s, q_star), atol=0))

        options3 = mut.IrisZoOptions()
        options3.parameterization = options2.parameterization
        self.assertFalse(
            options3.parameterization.get_parameterization_is_threadsafe())
        self.assertEqual(
            options3.parameterization.get_parameterization_dimension(), 2)
        self.assertTrue(
            callable(options3.parameterization.get_parameterization()))
        q3 = options3.parameterization.get_parameterization()(np.array(s))
        self.assertTrue(np.allclose(q3,
                                    kin.ComputeQValue(s, q_star), atol=0))

        options4 = mut.IrisZoOptions()
        v = Variable("v")
        options4.parameterization = IrisParameterizationFunction(
              expression_parameterization=[2 * v + 1], variables=[v])
        self.assertTrue(
            options4.parameterization.get_parameterization_is_threadsafe())
        self.assertEqual(
            options4.parameterization.get_parameterization_dimension(), 1)
        q3 = options4.parameterization.get_parameterization()(np.zeros(1))[0]
        self.assertEqual(q3, 2 * 0 + 1)
