import pydrake.planning as mut  # ruff: isort: skip

import textwrap
import unittest

import numpy as np

from pydrake.common import Parallelism, RandomGenerator
from pydrake.geometry.optimization import IrisOptions
from pydrake.planning import (
    IrisNp2Options,
    IrisZoOptions,
    RobotDiagramBuilder,
    SceneGraphCollisionChecker,
)
from pydrake.solvers import GurobiSolver, MosekSolver, SnoptSolver


def _snopt_and_mip_solver_available():
    has_mosek = MosekSolver().available() and MosekSolver().enabled()
    has_gurobi = GurobiSolver().available() and GurobiSolver().enabled()
    has_snopt = SnoptSolver().available() and SnoptSolver().enabled()
    has_mip_solver = has_mosek or has_gurobi
    return has_mip_solver and has_snopt


class TestIrisFromCliqueCover(unittest.TestCase):
    def _make_robot_diagram(self):
        # Code taken from
        # bindings/pydrake/planning/test/collision_checker_test.py
        builder = mut.RobotDiagramBuilder()
        scene_yaml = textwrap.dedent(
            """
        directives:
        - add_model:
            name: box
            file: package://drake/multibody/models/box.urdf
        - add_model:
            name: ground
            file: package://drake/planning/test_utilities/collision_ground_plane.sdf  # noqa
        - add_weld:
            parent: world
            child: ground::ground_plane_box
        """
        )
        builder.parser().AddModelsFromString(scene_yaml, "dmd.yaml")
        model_instance_index = builder.plant().GetModelInstanceByName("box")
        robot_diagram = builder.Build()
        return (robot_diagram, model_instance_index)

    def _make_scene_graph_collision_checker(self, use_provider, use_function):
        # Code taken from
        # bindings/pydrake/planning/test/collision_checker_test.py
        self.assertFalse(use_provider and use_function)

        robot, index = self._make_robot_diagram()
        plant = robot.plant()
        checker_kwargs = dict(
            model=robot, robot_model_instances=[index], edge_step_size=0.125
        )

        if use_provider:
            checker_kwargs["distance_and_interpolation_provider"] = (
                mut.LinearDistanceAndInterpolationProvider(plant)
            )
        if use_function:
            checker_kwargs["configuration_distance_function"] = (
                self._configuration_distance
            )

        return mut.SceneGraphCollisionChecker(**checker_kwargs)

    def test_iris_in_configuration_space_from_clique_cover_options(self):
        options = mut.IrisFromCliqueCoverOptions()
        options.iris_options.iteration_limit = 2
        self.assertEqual(options.iris_options.iteration_limit, 2)
        options.coverage_termination_threshold = 1e-3
        self.assertEqual(options.coverage_termination_threshold, 1e-3)
        options.iteration_limit = 10
        self.assertEqual(options.iteration_limit, 10)
        options.num_points_per_coverage_check = 100
        self.assertEqual(options.num_points_per_coverage_check, 100)
        options.parallelism = Parallelism(3)
        self.assertEqual(options.parallelism.num_threads(), 3)
        options.minimum_clique_size = 2
        self.assertEqual(options.minimum_clique_size, 2)
        options.num_points_per_visibility_round = 150
        self.assertEqual(options.num_points_per_visibility_round, 150)
        options.rank_tol_for_minimum_volume_circumscribed_ellipsoid = 1e-3
        self.assertEqual(
            options.rank_tol_for_minimum_volume_circumscribed_ellipsoid, 1e-3
        )
        options.point_in_set_tol = 1e-5
        self.assertEqual(options.point_in_set_tol, 1e-5)

        self.assertIsInstance(options.iris_options, IrisOptions)

        options.iris_options = IrisZoOptions()
        self.assertIsInstance(options.iris_options, IrisZoOptions)
        options.iris_options.sampled_iris_options.max_iterations = 2
        self.assertEqual(
            options.iris_options.sampled_iris_options.max_iterations, 2
        )

        options.iris_options = IrisNp2Options()
        self.assertIsInstance(options.iris_options, IrisNp2Options)
        options.iris_options.sampled_iris_options.max_iterations = 1
        self.assertEqual(
            options.iris_options.sampled_iris_options.max_iterations, 1
        )

    # IPOPT performs poorly on this test. We also need a MIP solver to
    # be available.  Hence only run this test if both SNOPT and a MIP solver
    # are available.
    @unittest.skipUnless(
        _snopt_and_mip_solver_available(), "Requires Snopt and a MIP solver"
    )
    def test_iris_in_configuration_space_from_clique_cover(self):
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
        params = dict(edge_step_size=0.125)
        builder = RobotDiagramBuilder()
        params["robot_model_instances"] = builder.parser().AddModelsFromString(
            cross_cspace_urdf, "urdf"
        )
        params["model"] = builder.Build()
        checker = SceneGraphCollisionChecker(**params)

        options = mut.IrisFromCliqueCoverOptions()
        options.num_points_per_coverage_check = 10
        options.num_points_per_visibility_round = 25

        # We can achieve almost 100% coverage with 2 regions.
        options.coverage_termination_threshold = 0.999
        options.iteration_limit = 3

        generator = RandomGenerator(0)

        sets = mut.IrisInConfigurationSpaceFromCliqueCover(
            checker=checker, options=options, generator=generator, sets=[]
        )
        self.assertGreaterEqual(len(sets), 2)

        class DummyMaxCliqueSolver(mut.MaxCliqueSolverBase):
            def __init__(self, name):
                mut.MaxCliqueSolverBase.__init__(self)
                self.name = name

            def DoSolveMaxClique(self, adjacency_matrix):
                return np.ones(adjacency_matrix.shape[1])

        # Check that a Python solver can be used with 1 thread.
        options.parallelism = Parallelism(1)
        sets = mut.IrisInConfigurationSpaceFromCliqueCover(
            checker=checker,
            options=options,
            generator=generator,
            sets=[],
            max_clique_solver=DummyMaxCliqueSolver("dummy"),
        )
        self.assertGreaterEqual(len(sets), 1)
