import pydrake.planning as mut

import copy
import textwrap
import unittest

import numpy as np
import scipy.sparse

from pydrake.common.test_utilities import numpy_compare
from pydrake.geometry import Sphere
from pydrake.math import RigidTransform
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import (
    BodyIndex,
    ModelInstanceIndex,
)
from pydrake.systems.framework import Context


class TestCollisionChecker(unittest.TestCase):
    """Tests the CollisionChecker API including all of its concrete subclasses
    as well as all of its related lower-level "accoutrements" classes.
    """

    def _make_robot_diagram(self):
        builder = mut.RobotDiagramBuilder()
        scene_yaml = textwrap.dedent("""
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
        """)
        builder.parser().AddModelsFromString(scene_yaml, "dmd.yaml")
        model_instance_index = builder.plant().GetModelInstanceByName("box")
        robot_diagram = builder.Build()
        return (robot_diagram, model_instance_index)

    def test_body_shape_description(self):
        # Constructor.
        dut = mut.BodyShapeDescription(
            shape=Sphere(0.25),
            X_BS=RigidTransform.Identity(),
            model_instance_name="foo",
            body_name="bar")

        # Getters.
        self.assertEqual(dut.shape().radius(), 0.25)
        self.assertIsInstance(dut.pose_in_body(), RigidTransform)
        self.assertEqual(dut.model_instance_name(), "foo")
        self.assertEqual(dut.body_name(), "bar")

        # Copyable.
        copy.copy(dut)
        mut.BodyShapeDescription(other=dut)

    def test_make_body_shape_description(self):
        robot, _ = self._make_robot_diagram()
        plant = robot.plant()
        (geometry_id,) = plant.GetCollisionGeometriesForBody(
            plant.GetBodyByName("box"))
        context = robot.CreateDefaultContext()
        dut = mut.MakeBodyShapeDescription(
            plant=plant,
            plant_context=robot.plant_context(context),
            geometry_id=geometry_id)
        self.assertIsInstance(dut, mut.BodyShapeDescription)
        self.assertEqual(dut.body_name(), "box")

    def test_linear_distance_interpolation_provider(self):
        robot, _ = self._make_robot_diagram()
        plant = robot.plant()

        mut.LinearDistanceAndInterpolationProvider(plant=plant)

        box_joint_index = plant.GetJointByName("box").index()
        box_joint_weights = np.array([1.0, 0.0, 0.0, 0.0, 2.0, 3.0, 4.0])
        joint_distance_weights = {box_joint_index: box_joint_weights}
        mut.LinearDistanceAndInterpolationProvider(
            plant=plant,
            joint_distance_weights=joint_distance_weights)

        distance_weights = np.array([1.0, 0.0, 0.0, 0.0, 2.0, 3.0, 4.0])
        weights_vector_provider = mut.LinearDistanceAndInterpolationProvider(
            plant=plant,
            distance_weights=distance_weights)
        numpy_compare.assert_equal(
            weights_vector_provider.distance_weights(),
            distance_weights)
        self.assertEqual(
            weights_vector_provider.quaternion_dof_start_indices(),
            [0])

    @staticmethod
    def _configuration_distance(q1, q2):
        """A boring implementation of ConfigurationDistanceFunction."""
        return np.linalg.norm(q1 - q2)

    def test_collision_checker_params(self):
        robot, index = self._make_robot_diagram()

        # Default constructor; write to properties.
        dut = mut.CollisionCheckerParams()
        dut.model = robot
        dut.robot_model_instances = [index]
        dut.configuration_distance_function = self._configuration_distance
        dut.edge_step_size = 0.125
        dut.env_collision_padding = 0.0625
        dut.self_collision_padding = 0.03125

        # Read from properties.
        self.assertIsInstance(dut.model.plant(), MultibodyPlant)
        self.assertListEqual(dut.robot_model_instances, [index])
        self.assertEqual(dut.configuration_distance_function(
            np.array([0.25]), np.array([0.75])), 0.5)
        self.assertEqual(dut.edge_step_size, 0.125)
        self.assertEqual(dut.env_collision_padding, 0.0625)
        self.assertEqual(dut.self_collision_padding, 0.03125)

        # ParamInit.
        dut = mut.CollisionCheckerParams(
            robot_model_instances=[index, index])
        self.assertEqual(len(dut.robot_model_instances), 2)

    def test_collision_checker_context(self):
        robot, _ = self._make_robot_diagram()

        # Constructor.
        dut = mut.CollisionCheckerContext(model=robot)

        # Getters.
        dut.model_context()
        dut.plant_context()
        dut.scene_graph_context()
        dut.GetQueryObject()

        # Clone.
        dut.Clone()
        copy.copy(dut)

    def test_edge_measure(self):
        # Constructor.
        dut = mut.EdgeMeasure(distance=0.25, alpha=1.0)

        # Getters.
        self.assertEqual(dut.completely_free(), True)
        self.assertEqual(dut.partially_free(), True)
        self.assertEqual(dut.distance(), 0.25)
        self.assertEqual(dut.alpha(), 1.0)
        self.assertEqual(dut.alpha_or(default_value=0.2), 1.0)

        # Copyable.
        copy.copy(dut)
        mut.EdgeMeasure(other=dut)

    def test_robot_clearance(self):
        # Constructor.
        dut = mut.RobotClearance(num_positions=3)

        # Getters (empty).
        self.assertEqual(dut.num_positions(), 3)
        self.assertEqual(dut.size(), 0)
        self.assertListEqual(dut.robot_indices(), [])
        self.assertListEqual(dut.other_indices(), [])
        self.assertListEqual(dut.collision_types(), [])
        numpy_compare.assert_equal(dut.distances(), [])
        numpy_compare.assert_equal(dut.jacobians(), [[]])
        numpy_compare.assert_equal(dut.mutable_jacobians(), [[]])

        # Setters.
        type_self = mut.RobotCollisionType.kSelfCollision
        type_env = mut.RobotCollisionType.kEnvironmentCollision
        dut.Reserve(size=2)
        dut.Append(
            robot_index=BodyIndex(10),
            other_index=BodyIndex(11),
            collision_type=type_self,
            distance=0.1,
            jacobian=np.array([0.0, 0.0, 0.01]))
        dut.Append(
            robot_index=BodyIndex(10),
            other_index=BodyIndex(22),
            collision_type=type_env,
            distance=0.2,
            jacobian=np.array([0.25, 0.00, 0.0]))

        # Getters (non-empty).
        self.assertEqual(dut.num_positions(), 3)
        self.assertEqual(dut.size(), 2)
        self.assertEqual(dut.robot_indices(), [BodyIndex(10), BodyIndex(10)])
        self.assertEqual(dut.other_indices(), [BodyIndex(11), BodyIndex(22)])
        self.assertEqual(dut.collision_types(), [type_self, type_env])
        numpy_compare.assert_equal(dut.distances(), [0.1, 0.2])
        expected_jacobians = np.array([
            [0.0, 0.0, 0.01],
            [0.25, 0.0, 0.0],
        ])
        numpy_compare.assert_equal(dut.jacobians(), expected_jacobians)
        numpy_compare.assert_equal(dut.mutable_jacobians(), expected_jacobians)

        # Mutation.
        dut.mutable_jacobians()[1, 2] = 0.5
        self.assertEqual(dut.jacobians()[1, 2], 0.5)

        # Copyable.
        copy.copy(dut)
        mut.RobotClearance(other=dut)

    def test_robot_collision_type(self):
        # Check that all values are bound.
        dut = mut.RobotCollisionType
        values = [
            dut.kNoCollision,
            dut.kEnvironmentCollision,
            dut.kSelfCollision,
            dut.kEnvironmentAndSelfCollision,
        ]

        # Calling MakeUpdated() with no arguments is a no-op.
        for x in values:
            updated = x.MakeUpdated()
            self.assertEqual(x, updated)

        # Helper function to create the enum via two bools.
        # (It would be nice if the Enum itself offered this directly.)
        def _make(env_col, self_col):
            if env_col and self_col:
                return dut.kEnvironmentAndSelfCollision
            if env_col:
                return dut.kEnvironmentCollision
            if self_col:
                return dut.kSelfCollision
            return dut.kNoCollision

        # Brute-force test all permutations of the Update method.
        for start_env in (False, True):
            for start_self in (False, True):
                start = _make(start_env, start_self)
                for update_env in (None, False, True):
                    for update_self in (None, False, True):
                        actual = start.MakeUpdated(
                            in_environment_collision=update_env,
                            in_self_collision=update_self)
                        expected = _make(
                            start_env if update_env is None else update_env,
                            start_self if update_self is None else update_self)
                        self.assertEqual(actual, expected)

    def _test_collision_checker_base_class(self, dut, has_provider):
        """Checks the API of CollisionChecker, given a concrete instance.
        """
        self.assertIsInstance(dut.model(), mut.RobotDiagram)
        self.assertIsInstance(dut.plant(), MultibodyPlant)

        body = dut.plant().GetBodyByName("box")
        frame = body.body_frame()
        env_body = dut.plant().GetBodyByName("ground_plane_box")
        num_bodies = 3

        self.assertIs(dut.get_body(body_index=body.index()), body)
        self.assertEqual(len(dut.robot_model_instances()), 1)
        self.assertTrue(dut.IsPartOfRobot(body=body))
        self.assertTrue(dut.IsPartOfRobot(body_index=body.index()))

        self.assertGreater(len(dut.GetZeroConfiguration()), 0)
        self.assertGreater(dut.num_allocated_contexts(), 0)
        self.assertIsInstance(dut.model_context(), mut.CollisionCheckerContext)
        self.assertIsInstance(dut.plant_context(), Context)
        self.assertIsInstance(
            dut.model_context(context_number=1), mut.CollisionCheckerContext)
        self.assertIsInstance(dut.plant_context(context_number=1), Context)

        q = np.array([0.25] * 7)
        self.assertIs(dut.UpdatePositions(q=q), dut.plant_context())
        self.assertIs(
            dut.UpdatePositions(q=q, context_number=1),
            dut.plant_context(context_number=1))
        ccc = dut.MakeStandaloneModelContext()  # ... a CollisionCheckerContext
        self.assertIsInstance(dut.UpdateContextPositions(
            model_context=ccc, q=q), Context)

        def operation(robot_diagram, context):
            self.assertIsInstance(robot_diagram, mut.RobotDiagram)
            self.assertIsInstance(context, mut.CollisionCheckerContext)

        dut.PerformOperationAgainstAllModelContexts(operation)

        X = RigidTransform.Identity()
        shape = Sphere(0.1)
        body_shape_description = mut.BodyShapeDescription(
            shape=shape, X_BS=X, model_instance_name="ground",
            body_name="ground_plane_box")
        dut.AddCollisionShape(
            group_name="foo", description=body_shape_description)
        dut.AddCollisionShapes(
            group_name="bar", descriptions=[body_shape_description])
        dut.AddCollisionShapes(
            geometry_groups={"baz": [body_shape_description]})
        dut.AddCollisionShapeToFrame(
            group_name="quux", frameA=frame, shape=shape, X_AG=X)
        dut.AddCollisionShapeToBody(
            group_name="quux", bodyA=body, shape=shape, X_AG=X)
        dut.GetAllAddedCollisionShapes()
        dut.RemoveAllAddedCollisionShapes(group_name="foo")
        dut.RemoveAllAddedCollisionShapes()

        dut.MaybeGetUniformRobotEnvironmentPadding()
        dut.MaybeGetUniformRobotRobotPadding()
        dut.GetPaddingBetween(bodyA_index=env_body.index(),
                              bodyB_index=body.index())
        dut.GetPaddingBetween(bodyA=env_body, bodyB=body)
        dut.SetPaddingBetween(bodyA_index=env_body.index(),
                              bodyB_index=body.index(),
                              padding=0.1)
        dut.SetPaddingBetween(bodyA=env_body, bodyB=body, padding=0.2)
        self.assertEqual(dut.GetPaddingMatrix().shape,
                         (num_bodies, num_bodies))
        dut.SetPaddingMatrix(collision_padding=np.zeros(
                                 (num_bodies, num_bodies)))
        dut.GetLargestPadding()
        dut.SetPaddingOneRobotBodyAllEnvironmentPairs(
            body_index=body.index(), padding=0.1)
        dut.SetPaddingAllRobotEnvironmentPairs(padding=0.1)
        dut.SetPaddingAllRobotRobotPairs(padding=0.1)

        self.assertEqual(dut.GetNominalFilteredCollisionMatrix().shape,
                         (num_bodies, num_bodies))
        self.assertEqual(dut.GetFilteredCollisionMatrix().shape,
                         (num_bodies, num_bodies))
        dut.SetCollisionFilterMatrix(
            filter_matrix=dut.GetNominalFilteredCollisionMatrix())
        dut.IsCollisionFilteredBetween(bodyA_index=env_body.index(),
                                       bodyB_index=body.index())
        dut.IsCollisionFilteredBetween(bodyA=env_body, bodyB=body)
        dut.SetCollisionFilteredBetween(
            bodyA_index=env_body.index(), bodyB_index=body.index(),
            filter_collision=True)
        dut.SetCollisionFilteredBetween(
            bodyA=env_body, bodyB=body, filter_collision=True)
        dut.SetCollisionFilteredWithAllBodies(body_index=body.index())
        dut.SetCollisionFilteredWithAllBodies(body=body)

        dut.CheckConfigCollisionFree(q=q)
        dut.CheckConfigCollisionFree(q=q, context_number=1)
        dut.CheckContextConfigCollisionFree(model_context=ccc, q=q)
        self.assertEqual(
            len(dut.CheckConfigsCollisionFree(
                configs=[q]*4, parallelize=False)),
            4)
        dut.CheckConfigsCollisionFree([q])  # Omit the defaulted arg.

        if not has_provider:
            def distance_function(q1, q2):
                return np.linalg.norm(q1 - q2)

            dut.SetConfigurationDistanceFunction(
                distance_function=distance_function)
            dut.ComputeConfigurationDistance(q1=q, q2=q)
            self.assertEqual(
                dut.MakeStandaloneConfigurationDistanceFunction()(q, q),
                0.0)

            def interpolation_function(q1, q2, r):
                return q1 + (q2 - q1) * r

            dut.SetConfigurationInterpolationFunction(
                interpolation_function=interpolation_function)
            dut.InterpolateBetweenConfigurations(q1=q, q2=q, ratio=0.5)
            numpy_compare.assert_equal(
                dut.MakeStandaloneConfigurationInterpolationFunction()(
                    q, q, 0.5),
                q)

        dut.edge_step_size()
        dut.set_edge_step_size(edge_step_size=0.2)
        dut.CheckEdgeCollisionFree(q1=q, q2=q)
        dut.CheckEdgeCollisionFree(q1=q, q2=q, context_number=1)
        dut.CheckContextEdgeCollisionFree(model_context=ccc, q1=q, q2=q)
        dut.CheckEdgeCollisionFreeParallel(q1=q, q2=q, parallelize=False)
        self.assertEqual(
            len(dut.CheckEdgesCollisionFree(
                edges=[(q, q)]*4,
                parallelize=False)),
            4)
        dut.CheckEdgesCollisionFree([(q, q)])  # Omit the defaulted arg.

        dut.MeasureEdgeCollisionFree(q1=q, q2=q)
        dut.MeasureEdgeCollisionFree(q1=q, q2=q, context_number=1)
        dut.MeasureContextEdgeCollisionFree(model_context=ccc, q1=q, q2=q)
        dut.MeasureEdgeCollisionFreeParallel(q1=q, q2=q, parallelize=False)
        measures = dut.MeasureEdgesCollisionFree(
            edges=[(q, q)]*4, parallelize=False)
        self.assertEqual(len(measures), 4)
        self.assertIsInstance(measures[0], mut.EdgeMeasure)
        dut.MeasureEdgesCollisionFree([(q, q)])  # Omit the defaulted arg.

        clearance = dut.CalcRobotClearance(q=q, influence_distance=10)
        self.assertIsInstance(clearance, mut.RobotClearance)
        clearance = dut.CalcRobotClearance(
            q=q, influence_distance=10, context_number=1)
        self.assertIsInstance(clearance, mut.RobotClearance)
        clearance = dut.CalcContextRobotClearance(
            model_context=ccc, q=q, influence_distance=10)
        self.assertIsInstance(clearance, mut.RobotClearance)

        dut.MaxNumDistances()
        dut.MaxNumDistances(context_number=1)
        dut.MaxContextNumDistances(model_context=ccc)

        dut.ClassifyBodyCollisions(q=q)
        dut.ClassifyBodyCollisions(q=q, context_number=1)
        dut.ClassifyContextBodyCollisions(model_context=ccc, q=q)

        self.assertIsInstance(dut.SupportsParallelChecking(), bool)

        provider = dut.distance_and_interpolation_provider()
        self.assertIsInstance(provider, mut.DistanceAndInterpolationProvider)
        new_provider = mut.LinearDistanceAndInterpolationProvider(
            dut.model().plant())
        dut.SetDistanceAndInterpolationProvider(provider=new_provider)

    def _make_scene_graph_collision_checker(self, use_provider, use_function):
        self.assertFalse(use_provider and use_function)

        robot, index = self._make_robot_diagram()
        plant = robot.plant()
        checker_kwargs = dict(
            model=robot,
            robot_model_instances=[index],
            edge_step_size=0.125)

        if use_provider:
            checker_kwargs["distance_and_interpolation_provider"] = \
                mut.LinearDistanceAndInterpolationProvider(plant)
        if use_function:
            checker_kwargs["configuration_distance_function"] = \
                self._configuration_distance

        return mut.SceneGraphCollisionChecker(**checker_kwargs)

    def test_scene_graph_collision_checker(self):
        """Tests the full CollisionChecker API.
        """
        # With no provider or function specified, the default SGCC has a
        # LinearDistanceAndInterpolationProvider.
        default_checker = self._make_scene_graph_collision_checker(
            False, False)
        self._test_collision_checker_base_class(default_checker, True)

        provider_checker = self._make_scene_graph_collision_checker(
            True, False)
        self._test_collision_checker_base_class(provider_checker, True)

        function_checker = self._make_scene_graph_collision_checker(
            False, True)
        self._test_collision_checker_base_class(function_checker, False)

    def test_visibility_graph(self):
        checker = self._make_scene_graph_collision_checker(True, False)
        plant = checker.model().plant()
        num_points = 2
        points = np.empty((plant.num_positions(), num_points))
        points[:, 0] = plant.GetPositions(checker.plant_context())
        points[:, 1] = points[:, 0]
        points[-1, 1] += 0.1
        A = mut.VisibilityGraph(checker=checker,
                                points=points,
                                parallelize=False)
        self.assertEqual(A.shape, (num_points, num_points))
        self.assertIsInstance(A, scipy.sparse.csc_matrix)
