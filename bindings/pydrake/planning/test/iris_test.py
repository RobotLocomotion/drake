import unittest

import pydrake.planning as mut
from pydrake.autodiffutils import AutoDiffXd
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
from pydrake.solvers import IpoptSolver
from pydrake.symbolic import Variable

import numpy as np

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


def SetSampledIrisOptions(options):
    options.sampled_iris_options.num_particles = 1000
    options.sampled_iris_options.tau = 0.5
    options.sampled_iris_options.delta = 5e-2
    options.sampled_iris_options.epsilon = 1e-2
    options.sampled_iris_options.\
        containment_points = np.array([[0, 0], [1, 0]])
    options.sampled_iris_options.max_iterations = 1
    options.sampled_iris_options.max_iterations_separating_planes = 20
    options.sampled_iris_options.max_separating_planes_per_iteration = 10
    options.sampled_iris_options.parallelism = Parallelism(True)
    options.sampled_iris_options.verbose = False
    options.sampled_iris_options.configuration_space_margin = 1e-2
    options.sampled_iris_options.relax_margin = False
    options.sampled_iris_options.termination_threshold = 1e-2
    options.sampled_iris_options.relative_termination_threshold = 1e-3
    options.sampled_iris_options.random_seed = 1337
    options.sampled_iris_options.mixing_steps = 50
    options.sampled_iris_options.sample_particles_in_parallel = False
    options.sampled_iris_options.remove_all_collisions_possible = True


class TestIrisZo(unittest.TestCase):
    def test_iris_zo(self):
        params = dict(edge_step_size=0.1)
        builder = RobotDiagramBuilder()
        params["robot_model_instances"] = builder.parser().AddModelsFromString(
            cross_cspace_urdf, "urdf"
        )
        params["model"] = builder.Build()
        plant = params["model"].plant()
        checker = SceneGraphCollisionChecker(**params)
        options = mut.IrisZoOptions()
        options.bisection_steps = 10
        options.sampled_iris_options.\
            prog_with_additional_constraints = InverseKinematics(
                plant
            ).prog()
        SetSampledIrisOptions(options)
        domain = HPolyhedron.MakeBox(plant.GetPositionLowerLimits(),
                                     plant.GetPositionUpperLimits())
        starting_ellipsoid = Hyperellipsoid.MakeHypersphere(0.01,
                                                            np.zeros((2,)))
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
            callable(options.parameterization.get_parameterization_double()))
        s = np.array([0, 1])
        q = options.parameterization.get_parameterization_double()(s)
        self.assertTrue(np.allclose(q,
                                    kin.ComputeQValue(s, q_star), atol=0))

        options2 = mut.IrisZoOptions()
        options2.parameterization = IrisParameterizationFunction(
            options.parameterization.get_parameterization_double(),
            options.parameterization.get_parameterization_dimension())
        self.assertFalse(
            options2.parameterization.get_parameterization_is_threadsafe())
        self.assertEqual(
            options2.parameterization.get_parameterization_dimension(), 2)
        self.assertTrue(
            callable(options2.parameterization.get_parameterization_double()))
        q2 = options2.parameterization.get_parameterization_double()(
            np.array(s))
        self.assertTrue(np.allclose(q2,
                                    kin.ComputeQValue(s, q_star), atol=0))

        options3 = mut.IrisZoOptions()
        options3.parameterization = options2.parameterization
        self.assertFalse(
            options3.parameterization.get_parameterization_is_threadsafe())
        self.assertEqual(
            options3.parameterization.get_parameterization_dimension(), 2)
        self.assertTrue(
            callable(options3.parameterization.get_parameterization_double()))
        q3 = options3.parameterization.get_parameterization_double()(
            np.array(s))
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
        q3 = options4.parameterization.get_parameterization_double()(
            np.zeros(1))[0]
        self.assertEqual(q3, 2 * 0 + 1)


class TestIrisNp2(unittest.TestCase):
    def test_iris_np2(self):
        params = dict(edge_step_size=0.1)
        builder = RobotDiagramBuilder()
        params["robot_model_instances"] = builder.parser().AddModelsFromString(
            cross_cspace_urdf, "urdf"
        )
        params["model"] = builder.Build()
        plant = params["model"].plant()
        checker = SceneGraphCollisionChecker(**params)
        options = mut.IrisNp2Options()
        SetSampledIrisOptions(options)
        options.sampling_strategy = "greedy"
        options.ray_sampler_options.only_walk_toward_collisions = True
        options.ray_sampler_options.ray_search_num_steps = 10
        options.ray_sampler_options.num_particles_to_walk_towards = 200
        options.add_hyperplane_if_solve_fails = False

        # For speed reasons -- IPOPT seems to be faster than SNOPT here.
        options.solver = IpoptSolver()

        domain = HPolyhedron.MakeBox(plant.GetPositionLowerLimits(),
                                     plant.GetPositionUpperLimits())
        starting_ellipsoid = Hyperellipsoid.MakeHypersphere(0.01,
                                                            np.zeros((2,)))
        region = mut.IrisNp2(checker=checker,
                             starting_ellipsoid=starting_ellipsoid,
                             domain=domain,
                             options=options)
        test_point = np.array([0.0, 0.5])
        self.assertTrue(region.PointInSet(test_point))

        def parameterization_function(q):
            return 2.0 * q + 1.0
        double_input = np.zeros(2)
        autodiff_input = np.array(2 * [AutoDiffXd(0.0)])
        double_output = parameterization_function(double_input)
        autodiff_output = parameterization_function(autodiff_input)
        self.assertTrue(isinstance(double_output[0], float))
        self.assertTrue(isinstance(autodiff_output[0], AutoDiffXd))

        options.parameterization = IrisParameterizationFunction(
            parameterization=parameterization_function, dimension=2)

        def inverse_parameterization(q):
            return (q - 1.0) / 2.0

        # The domain in the original coordinates is from -2 to 2 in each
        # dimension, so in the new coordinates, it's from -1.5 to 0.5 in each
        # dimension.
        domain_shifted = HPolyhedron.MakeBox([-1.5, -1.5], [0.5, 0.5])
        starting_ellipsoid_shifted = Hyperellipsoid.MakeHypersphere(
            0.01, inverse_parameterization(starting_ellipsoid.center()))

        # We also need to shift the containment points.
        new_containment_points = options.sampled_iris_options.\
            containment_points.copy()
        for i in range(new_containment_points.shape[1]):
            new_containment_points[:, i] = inverse_parameterization(
                new_containment_points[:, i])
        options.sampled_iris_options.\
            containment_points = new_containment_points
        region = mut.IrisNp2(checker=checker,
                             starting_ellipsoid=starting_ellipsoid_shifted,
                             domain=domain_shifted,
                             options=options)
        test_point_shifted = inverse_parameterization(test_point)
        self.assertTrue(region.PointInSet(test_point_shifted))


class TestOptionsPrinting(unittest.TestCase):
    def test_options_printing(self):
        options_zo = mut.IrisZoOptions()
        options_np2 = mut.IrisNp2Options()

        s_zo = repr(options_zo)
        self.assertTrue(
                ("bisection_steps=%d" % options_zo.bisection_steps) in s_zo)

        s_np2 = repr(options_np2)
        self.assertTrue(
                ("sampling_strategy=%s" %
                 options_np2.sampling_strategy) in s_np2)
        self.assertTrue(
                ("only_walk_toward_collisions=%r" %
                 options_np2.ray_sampler_options.
                 only_walk_toward_collisions) in s_np2)
        self.assertTrue(
                ("ray_search_num_steps=%d" %
                 options_np2.ray_sampler_options.
                 ray_search_num_steps) in s_np2)
        self.assertTrue(
                ("num_particles_to_walk_towards=%d" %
                 options_np2.ray_sampler_options.
                 num_particles_to_walk_towards) in s_np2)
        self.assertTrue(
                ("add_hyperplane_if_solve_fails=%r" %
                 options_np2.add_hyperplane_if_solve_fails) in s_np2)

        for options in [options_zo, options_np2]:
            s = repr(options)
            print(s)
            print("tau=%f" % options.sampled_iris_options.tau)
            self.assertTrue(
                    ("num_particles=%d" %
                     options.sampled_iris_options.num_particles) in s)
            self.assertTrue(
                    ("tau=%g" % options.sampled_iris_options.tau) in s)
            self.assertTrue(
                    ("delta=%g" % options.sampled_iris_options.delta) in s)
            self.assertTrue(
                    ("epsilon=%g" % options.sampled_iris_options.epsilon) in s)
            self.assertTrue(
                    ("max_iterations=%d" %
                     options.sampled_iris_options.max_iterations) in s)
            self.assertTrue(
                    ("max_iterations_separating_planes=%d" %
                     options.sampled_iris_options.
                     max_iterations_separating_planes) in s)
            self.assertTrue(
                    ("max_separating_planes_per_iteration=%d" %
                     options.sampled_iris_options.
                     max_separating_planes_per_iteration) in s)
            self.assertTrue(
                    ("parallelism=%s" %
                     options.sampled_iris_options.parallelism) in s)
            self.assertTrue(
                    ("verbose=%r" % options.sampled_iris_options.verbose) in s)
            self.assertTrue(
                    ("require_sample_point_is_contained=%r" %
                     options.sampled_iris_options.
                     require_sample_point_is_contained) in s)
            self.assertTrue(
                    ("configuration_space_margin=%g" %
                     options.sampled_iris_options.
                     configuration_space_margin) in s)
            self.assertTrue(
                    ("termination_threshold=%g" %
                     options.sampled_iris_options.termination_threshold) in s)
            self.assertTrue(
                    ("relative_termination_threshold=%g" %
                     options.sampled_iris_options.
                     relative_termination_threshold) in s)
            self.assertTrue(
                    ("remove_all_collisions_possible=%r" %
                     options.sampled_iris_options.
                     remove_all_collisions_possible) in s)
            self.assertTrue(
                    ("random_seed=%d" %
                     options.sampled_iris_options.random_seed) in s)
            self.assertTrue(
                    ("mixing_steps=%d" %
                     options.sampled_iris_options.mixing_steps) in s)
            self.assertTrue(
                    ("sample_particles_in_parallel=%r" %
                     options.sampled_iris_options.
                     sample_particles_in_parallel) in s)
