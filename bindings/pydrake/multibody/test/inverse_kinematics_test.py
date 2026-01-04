from pydrake.multibody import inverse_kinematics as ik  # ruff: isort: skip

from collections import namedtuple
from functools import partial, wraps
import math
import textwrap
import typing
import unittest

import numpy as np

import pydrake
from pydrake.common import FindResourceOrThrow
from pydrake.common.eigen_geometry import Quaternion
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.planning import RobotDiagramBuilder, SceneGraphCollisionChecker
import pydrake.solvers as mp
from pydrake.symbolic import Variable
from pydrake.systems.framework import DiagramBuilder

# TODO(eric.cousineau): Replace manual coordinate indexing with more semantic
# operations (`CalcRelativeTransform`, `SetFreeBodyPose`).


class TestInverseKinematics(unittest.TestCase):
    """
    This test reflects inverse_kinematics_test.cc
    """

    def setUp(self):
        builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            builder, MultibodyPlant(time_step=0.01)
        )
        Parser(self.plant).AddModels(
            FindResourceOrThrow(
                "drake/bindings/pydrake/multibody/test/two_bodies.sdf"
            )
        )
        self.plant.Finalize()
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        plant_context = diagram.GetMutableSubsystemContext(
            self.plant, diagram_context
        )
        self.body1_frame = self.plant.GetBodyByName("body1").body_frame()
        self.body2_frame = self.plant.GetBodyByName("body2").body_frame()
        self.ik_two_bodies = ik.InverseKinematics(
            plant=self.plant, plant_context=plant_context
        )
        # Test non-SceneGraph constructor.
        ik.InverseKinematics(plant=self.plant)
        self.prog = self.ik_two_bodies.get_mutable_prog()
        self.q = self.ik_two_bodies.q()

        # Test constructor without joint limits
        ik.InverseKinematics(plant=self.plant, with_joint_limits=False)
        ik.InverseKinematics(
            plant=self.plant,
            plant_context=plant_context,
            with_joint_limits=False,
        )

        def squaredNorm(x):
            return np.array([x[0] ** 2 + x[1] ** 2 + x[2] ** 2 + x[3] ** 2])

        self.prog.AddConstraint(squaredNorm, [1], [1], self._body1_quat(self.q))
        self.prog.AddConstraint(squaredNorm, [1], [1], self._body2_quat(self.q))
        self.prog.SetInitialGuess(self._body1_quat(self.q), [1, 0, 0, 0])
        self.prog.SetInitialGuess(self._body2_quat(self.q), [1, 0, 0, 0])

    def _body1_quat(self, q):
        # TODO(eric.cousineau): Replace with state indexing.
        return q[0:4]

    def _body1_xyz(self, q):
        # TODO(eric.cousineau): Replace with state indexing.
        return q[4:7]

    def _body2_quat(self, q):
        # TODO(eric.cousineau): Replace with state indexing.
        return q[7:11]

    def _body2_xyz(self, q):
        # TODO(eric.cousineau): Replace with state indexing.
        return q[11:14]

    def test_AddMultibodyPlantConstraints(self):
        bindings = ik.AddMultibodyPlantConstraints(
            plant=self.plant,
            q=self.q,
            prog=self.prog,
            plant_context=self.ik_two_bodies.context(),
        )
        self.assertEqual(len(bindings), 3)

    def test_AddPositionConstraint1(self):
        p_BQ = np.array([0.2, 0.3, 0.5])
        p_AQ_lower = np.array([-0.1, -0.2, -0.3])
        p_AQ_upper = np.array([-0.05, -0.12, -0.28])

        self.ik_two_bodies.AddPositionConstraint(
            frameB=self.body1_frame,
            p_BQ=p_BQ,
            frameA=self.body2_frame,
            p_AQ_lower=p_AQ_lower,
            p_AQ_upper=p_AQ_upper,
        )
        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        q_val = result.GetSolution(self.q)

        body1_quat = self._body1_quat(q_val)
        body1_pos = self._body1_xyz(q_val)
        body2_quat = self._body2_quat(q_val)
        body2_pos = self._body2_xyz(q_val)
        body1_rotmat = Quaternion(body1_quat).rotation()
        body2_rotmat = Quaternion(body2_quat).rotation()
        p_AQ = body2_rotmat.transpose().dot(
            body1_rotmat.dot(p_BQ) + body1_pos - body2_pos
        )
        self.assertTrue(
            np.greater(p_AQ, p_AQ_lower - 1e-6 * np.ones((3, 1))).all()
        )
        self.assertTrue(
            np.less(p_AQ, p_AQ_upper + 1e-6 * np.ones((3, 1))).all()
        )

        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        self.assertTrue(np.allclose(result.GetSolution(self.q), q_val))
        np.testing.assert_array_equal(
            self.plant.GetPositions(self.ik_two_bodies.context()), q_val
        )
        self.assertIs(
            self.ik_two_bodies.get_mutable_context(),
            self.ik_two_bodies.context(),
        )

    def test_AddPositionConstraint2(self):
        p_BQ = np.array([0.2, 0.3, 0.5])
        p_AQ_lower = np.array([-0.1, -0.2, -0.3])
        p_AQ_upper = np.array([-0.05, -0.12, -0.28])
        X_AbarA = RigidTransform([-0.1, -0.2, -0.3])

        self.ik_two_bodies.AddPositionConstraint(
            frameB=self.body1_frame,
            p_BQ=p_BQ,
            frameAbar=self.body2_frame,
            X_AbarA=X_AbarA,
            p_AQ_lower=p_AQ_lower,
            p_AQ_upper=p_AQ_upper,
        )
        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        q_val = result.GetSolution(self.q)

        body1_quat = self._body1_quat(q_val)
        body1_pos = self._body1_xyz(q_val)
        body2_quat = self._body2_quat(q_val)
        body2_pos = self._body2_xyz(q_val)
        body1_rotmat = Quaternion(body1_quat).rotation()
        body2_rotmat = Quaternion(body2_quat).rotation()
        p_AbarQ = body2_rotmat.transpose().dot(
            body1_rotmat.dot(p_BQ) + body1_pos - body2_pos
        )
        p_AQ = X_AbarA.inverse() @ p_AbarQ
        self.assertTrue(
            np.greater(p_AQ, p_AQ_lower - 1e-6 * np.ones((3, 1))).all()
        )
        self.assertTrue(
            np.less(p_AQ, p_AQ_upper + 1e-6 * np.ones((3, 1))).all()
        )

        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        self.assertTrue(np.allclose(result.GetSolution(self.q), q_val))
        np.testing.assert_array_equal(
            self.plant.GetPositions(self.ik_two_bodies.context()), q_val
        )
        self.assertIs(
            self.ik_two_bodies.get_mutable_context(),
            self.ik_two_bodies.context(),
        )

    def test_AddPositionCost(self):
        p_BQ = np.array([0.2, 0.3, 0.5])
        p_AP = np.array([-0.1, -0.2, -0.3])

        binding = self.ik_two_bodies.AddPositionCost(
            frameA=self.body1_frame,
            p_AP=p_AP,
            frameB=self.body2_frame,
            p_BQ=p_BQ,
            C=np.eye(3),
        )
        self.assertIsInstance(binding, mp.Binding[mp.Cost])

    def test_AddOrientationConstraint(self):
        theta_bound = 0.2 * math.pi
        R_AbarA = RotationMatrix(quaternion=Quaternion(0.5, -0.5, 0.5, 0.5))
        R_BbarB = RotationMatrix(
            quaternion=Quaternion(1.0 / 3, 2.0 / 3, 0, 2.0 / 3)
        )
        self.ik_two_bodies.AddOrientationConstraint(
            frameAbar=self.body1_frame,
            R_AbarA=R_AbarA,
            frameBbar=self.body2_frame,
            R_BbarB=R_BbarB,
            theta_bound=theta_bound,
        )

        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        q_val = result.GetSolution(self.q)

        body1_quat = self._body1_quat(q_val)
        body2_quat = self._body2_quat(q_val)
        body1_rotmat = Quaternion(body1_quat).rotation()
        body2_rotmat = Quaternion(body2_quat).rotation()
        R_AbarBbar = body1_rotmat.transpose().dot(body2_rotmat)
        R_AB = (
            R_AbarA.matrix().transpose().dot(R_AbarBbar.dot(R_BbarB.matrix()))
        )
        self.assertGreater(R_AB.trace(), 1 + 2 * math.cos(theta_bound) - 1e-6)

        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        q_val = result.GetSolution(self.q)

        body1_quat = self._body1_quat(q_val)
        body2_quat = self._body2_quat(q_val)
        body1_rotmat = Quaternion(body1_quat).rotation()
        body2_rotmat = Quaternion(body2_quat).rotation()
        R_AbarBbar = body1_rotmat.transpose().dot(body2_rotmat)
        R_AB = (
            R_AbarA.matrix().transpose().dot(R_AbarBbar.dot(R_BbarB.matrix()))
        )
        self.assertGreater(R_AB.trace(), 1 + 2 * math.cos(theta_bound) - 1e-6)

    def test_AddOrientationCost(self):
        binding = self.ik_two_bodies.AddOrientationCost(
            frameAbar=self.body1_frame,
            R_AbarA=RotationMatrix(),
            frameBbar=self.body2_frame,
            R_BbarB=RotationMatrix(),
            c=1.0,
        )
        self.assertIsInstance(binding, mp.Binding[mp.Cost])

    def test_AddGazeTargetConstraint(self):
        p_AS = np.array([0.1, 0.2, 0.3])
        n_A = np.array([0.3, 0.5, 1.2])
        p_BT = np.array([1.1, 0.2, 1.5])
        cone_half_angle = 0.2 * math.pi

        self.ik_two_bodies.AddGazeTargetConstraint(
            frameA=self.body1_frame,
            p_AS=p_AS,
            n_A=n_A,
            frameB=self.body2_frame,
            p_BT=p_BT,
            cone_half_angle=cone_half_angle,
        )

        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        q_val = result.GetSolution(self.q)

        body1_quat = self._body1_quat(q_val)
        body1_pos = self._body1_xyz(q_val)
        body2_quat = self._body2_quat(q_val)
        body2_pos = self._body2_xyz(q_val)
        body1_rotmat = Quaternion(body1_quat).rotation()
        body2_rotmat = Quaternion(body2_quat).rotation()

        p_WS = body1_pos + body1_rotmat.dot(p_AS)
        p_WT = body2_pos + body2_rotmat.dot(p_BT)
        p_ST_W = p_WT - p_WS
        n_W = body1_rotmat.dot(n_A)
        self.assertGreater(
            p_ST_W.dot(n_W),
            np.linalg.norm(p_ST_W)
            * np.linalg.norm(n_W)
            * math.cos(cone_half_angle)
            - 1e-6,
        )

        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        q_val = result.GetSolution(self.q)

        body1_quat = self._body1_quat(q_val)
        body1_pos = self._body1_xyz(q_val)
        body2_quat = self._body2_quat(q_val)
        body2_pos = self._body2_xyz(q_val)
        body1_rotmat = Quaternion(body1_quat).rotation()
        body2_rotmat = Quaternion(body2_quat).rotation()

        p_WS = body1_pos + body1_rotmat.dot(p_AS)
        p_WT = body2_pos + body2_rotmat.dot(p_BT)
        p_ST_W = p_WT - p_WS
        n_W = body1_rotmat.dot(n_A)
        self.assertGreater(
            p_ST_W.dot(n_W),
            (
                np.linalg.norm(p_ST_W)
                * np.linalg.norm(n_W)
                * math.cos(cone_half_angle)
            )
            - 1e-6,
        )

    def test_AddAngleBetweenVectorsConstraint(self):
        na_A = np.array([0.2, -0.4, 0.9])
        nb_B = np.array([1.4, -0.1, 1.8])

        angle_lower = 0.2 * math.pi
        angle_upper = 0.2 * math.pi

        self.ik_two_bodies.AddAngleBetweenVectorsConstraint(
            frameA=self.body1_frame,
            na_A=na_A,
            frameB=self.body2_frame,
            nb_B=nb_B,
            angle_lower=angle_lower,
            angle_upper=angle_upper,
        )
        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())

        q_val = result.GetSolution(self.q)
        body1_quat = self._body1_quat(q_val)
        body2_quat = self._body2_quat(q_val)
        body1_rotmat = Quaternion(body1_quat).rotation()
        body2_rotmat = Quaternion(body2_quat).rotation()

        na_W = body1_rotmat.dot(na_A)
        nb_W = body2_rotmat.dot(nb_B)

        angle = math.acos(
            na_W.transpose().dot(nb_W)
            / (np.linalg.norm(na_W) * np.linalg.norm(nb_W))
        )

        self.assertLess(math.fabs(angle - angle_lower), 1e-6)

        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        self.assertTrue(np.allclose(result.GetSolution(self.q), q_val))

    def test_AddAngleBetweenVectorsCost(self):
        na_A = np.array([0.2, -0.4, 0.9])
        nb_B = np.array([1.4, -0.1, 1.8])

        self.ik_two_bodies.AddAngleBetweenVectorsCost(
            frameA=self.body1_frame,
            na_A=na_A,
            frameB=self.body2_frame,
            nb_B=nb_B,
            c=10.0,
        )

    def test_AddPointToPointDistanceConstraint(self):
        p_B1P1 = np.array([0.2, -0.4, 0.9])
        p_B2P2 = np.array([1.4, -0.1, 1.8])

        distance_lower = 0.1
        distance_upper = 0.2

        self.ik_two_bodies.AddPointToPointDistanceConstraint(
            frame1=self.body1_frame,
            p_B1P1=p_B1P1,
            frame2=self.body2_frame,
            p_B2P2=p_B2P2,
            distance_lower=distance_lower,
            distance_upper=distance_upper,
        )
        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())

        q_val = result.GetSolution(self.q)
        body1_quat = self._body1_quat(q_val)
        body2_quat = self._body2_quat(q_val)
        body1_rotmat = Quaternion(body1_quat).rotation()
        body2_rotmat = Quaternion(body2_quat).rotation()

        p_WP1 = self._body1_xyz(q_val) + body1_rotmat.dot(p_B1P1)
        p_WP2 = self._body2_xyz(q_val) + body2_rotmat.dot(p_B2P2)
        distance = np.linalg.norm(p_WP1 - p_WP2)

        self.assertLess(distance, distance_upper + 3e-6)
        self.assertGreater(distance, distance_lower - 3e-6)

        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        self.assertTrue(np.allclose(result.GetSolution(self.q), q_val))

    def test_AddPointToLineDistanceConstraint(self):
        p_B1P = np.array([0.2, -0.4, 0.9])
        p_B2Q = np.array([1.4, -0.1, 1.8])
        n_B2 = np.array([0.1, 0.3, 0.2])

        distance_lower = 0.1
        distance_upper = 0.2

        self.ik_two_bodies.AddPointToLineDistanceConstraint(
            frame_point=self.body1_frame,
            p_B1P=p_B1P,
            frame_line=self.body2_frame,
            p_B2Q=p_B2Q,
            n_B2=n_B2,
            distance_lower=distance_lower,
            distance_upper=distance_upper,
        )
        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())

        q_val = result.GetSolution(self.q)
        body1_quat = self._body1_quat(q_val)
        body2_quat = self._body2_quat(q_val)
        body1_rotmat = Quaternion(body1_quat).rotation()
        body2_rotmat = Quaternion(body2_quat).rotation()

        p_WP = self._body1_xyz(q_val) + body1_rotmat.dot(p_B1P)
        p_WQ = self._body2_xyz(q_val) + body2_rotmat.dot(p_B2Q)
        n_W = body2_rotmat @ n_B2
        n_W_normalized = n_W / np.linalg.norm(n_W)
        distance = np.linalg.norm(
            p_WQ + n_W_normalized.dot(p_WP - p_WQ) * n_W_normalized - p_WP
        )

        self.assertLess(distance, distance_upper + 1e-5)
        self.assertGreater(distance, distance_lower - 1e-5)

        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        self.assertTrue(np.allclose(result.GetSolution(self.q), q_val))

    def test_AddPolyhedronConstraint(self):
        p_GP = np.array([[0.2, -0.4], [0.9, 0.2], [-0.1, 1]])
        A = np.array([[0.5, 1.0, 0.1, 0.2, 0.5, 1.5]])
        b = np.array([10.0])

        self.ik_two_bodies.AddPolyhedronConstraint(
            frameF=self.body1_frame,
            frameG=self.body2_frame,
            p_GP=p_GP,
            A=A,
            b=b,
        )
        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())

    def test_AddMinimumDistanceLowerBoundConstraint(self):
        ik = self.ik_two_bodies
        W = self.plant.world_frame()
        B1 = self.body1_frame
        B2 = self.body2_frame

        min_distance_lower = 0.1
        tol = 1e-2
        radius1 = 0.1
        radius2 = 0.2

        ik.AddMinimumDistanceLowerBoundConstraint(bound=min_distance_lower)
        context = self.plant.CreateDefaultContext()
        self.plant.SetFreeBodyPose(
            context, B1.body(), RigidTransform([0, 0, 0.01])
        )
        self.plant.SetFreeBodyPose(
            context, B2.body(), RigidTransform([0, 0, -0.01])
        )

        def get_min_distance_actual():
            X = partial(self.plant.CalcRelativeTransform, context)
            distance = np.linalg.norm(
                X(W, B1).translation() - X(W, B2).translation()
            )
            return distance - radius1 - radius2

        self.assertLess(get_min_distance_actual(), min_distance_lower - tol)
        self.prog.SetInitialGuess(ik.q(), self.plant.GetPositions(context))
        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        q_val = result.GetSolution(ik.q())
        self.plant.SetPositions(context, q_val)
        self.assertGreater(get_min_distance_actual(), min_distance_lower - tol)

        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        self.assertTrue(np.allclose(result.GetSolution(ik.q()), q_val))

    def test_AddMinimumDistanceUpperBoundConstraint(self):
        ik = self.ik_two_bodies
        ik.AddMinimumDistanceUpperBoundConstraint(
            bound=0.1, influence_distance_offset=0.01
        )

    def test_AddDistanceConstraint(self):
        ik = self.ik_two_bodies
        W = self.plant.world_frame()
        B1 = self.body1_frame
        B2 = self.body2_frame

        distance_lower = 0.1
        distance_upper = 0.2
        tol = 1e-2

        radius1 = 0.1
        radius2 = 0.2

        inspector = self.scene_graph.model_inspector()
        frame_id1 = inspector.GetGeometryIdByName(
            self.plant.GetBodyFrameIdOrThrow(
                self.plant.GetBodyByName("body1").index()
            ),
            pydrake.geometry.Role.kProximity,
            "two_bodies::body1_collision",
        )
        frame_id2 = inspector.GetGeometryIdByName(
            self.plant.GetBodyFrameIdOrThrow(
                self.plant.GetBodyByName("body2").index()
            ),
            pydrake.geometry.Role.kProximity,
            "two_bodies::body2_collision",
        )
        ik.AddDistanceConstraint(
            geometry_pair=(frame_id1, frame_id2),
            distance_lower=distance_lower,
            distance_upper=distance_upper,
        )

        context = self.plant.CreateDefaultContext()
        self.plant.SetFreeBodyPose(
            context, B1.body(), RigidTransform([0, 0, 0.01])
        )
        self.plant.SetFreeBodyPose(
            context, B2.body(), RigidTransform([0, 0, -0.01])
        )

        def get_min_distance_actual():
            X = partial(self.plant.CalcRelativeTransform, context)
            distance = np.linalg.norm(
                X(W, B1).translation() - X(W, B2).translation()
            )
            return distance - radius1 - radius2

        self.assertLess(get_min_distance_actual(), distance_lower - tol)
        self.prog.SetInitialGuess(ik.q(), self.plant.GetPositions(context))
        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        q_val = result.GetSolution(ik.q())
        self.plant.SetPositions(context, q_val)
        self.assertGreater(get_min_distance_actual(), distance_lower - tol)
        self.assertLess(get_min_distance_actual(), distance_upper + tol)

        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        self.assertTrue(np.allclose(result.GetSolution(ik.q()), q_val))

    def test_AddUnitQuaternionConstraintOnPlant(self):
        prog = mp.MathematicalProgram()
        q = prog.NewContinuousVariables(self.plant.num_positions())
        ik.AddUnitQuaternionConstraintOnPlant(self.plant, q, prog)
        plant_ad = self.plant.ToAutoDiffXd()
        ik.AddUnitQuaternionConstraintOnPlant(plant_ad, q, prog)


class TestConstraints(unittest.TestCase):
    """
    This test partially reflects distance_constraint_test.cc.
    Currently, all tests are simple constructions tests.
    """

    def setUp(self):
        builder_f = DiagramBuilder()
        self.plant_f, self.scene_graph_f = AddMultibodyPlantSceneGraph(
            builder_f, MultibodyPlant(time_step=0.01)
        )
        Parser(self.plant_f).AddModels(
            FindResourceOrThrow(
                "drake/bindings/pydrake/multibody/test/two_bodies.sdf"
            )
        )
        self.plant_f.Finalize()
        diagram_f = builder_f.Build()
        diagram_ad = diagram_f.ToAutoDiffXd()
        plant_ad = diagram_ad.GetSubsystemByName(self.plant_f.get_name())

        TypeVariables = namedtuple(
            "TypeVariables",
            ("plant", "plant_context", "body1_frame", "body2_frame"),
        )

        def make_type_variables(plant_T, diagram_T):
            diagram_context_T = diagram_T.CreateDefaultContext()
            return TypeVariables(
                plant=plant_T,
                plant_context=diagram_T.GetMutableSubsystemContext(
                    plant_T, diagram_context_T
                ),
                body1_frame=plant_T.GetBodyByName("body1").body_frame(),
                body2_frame=plant_T.GetBodyByName("body2").body_frame(),
            )

        self.variables_f = make_type_variables(self.plant_f, diagram_f)
        self.variables_ad = make_type_variables(plant_ad, diagram_ad)

    def check_type_variables(check_method):
        @wraps(check_method)
        def wrapper(self):
            check_method(self, self.variables_f)
            check_method(self, self.variables_ad)

        return wrapper

    @check_type_variables
    def test_angle_between_vectors_constraint(self, variables):
        constraint = ik.AngleBetweenVectorsConstraint(
            plant=variables.plant,
            frameA=variables.body1_frame,
            a_A=[0.2, -0.4, 0.9],
            frameB=variables.body2_frame,
            b_B=[1.4, -0.1, 1.8],
            angle_lower=0.1 * math.pi,
            angle_upper=0.2 * math.pi,
            plant_context=variables.plant_context,
        )
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_angle_between_vectors_cost(self, variables):
        cost = ik.AngleBetweenVectorsCost(
            plant=variables.plant,
            frameA=variables.body1_frame,
            a_A=[0.2, -0.4, 0.9],
            frameB=variables.body2_frame,
            b_B=[1.4, -0.1, 1.8],
            c=10.0,
            plant_context=variables.plant_context,
        )
        self.assertIsInstance(cost, mp.Cost)

    @check_type_variables
    def test_distance_constraint_1(self, variables):
        def get_sphere_geometry_id(frame):
            (id_,) = variables.plant.GetCollisionGeometriesForBody(frame.body())
            return id_

        constraint = ik.DistanceConstraint(
            plant=variables.plant,
            geometry_pair=(
                get_sphere_geometry_id(variables.body1_frame),
                get_sphere_geometry_id(variables.body2_frame),
            ),
            plant_context=variables.plant_context,
            distance_lower=0.1,
            distance_upper=2,
        )
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_gaze_target_constraint(self, variables):
        constraint = ik.GazeTargetConstraint(
            plant=variables.plant,
            frameA=variables.body1_frame,
            p_AS=[0.1, 0.2, 0.3],
            n_A=[0.3, 0.5, 1.2],
            frameB=variables.body2_frame,
            p_BT=[1.1, 0.2, 1.5],
            cone_half_angle=0.2 * math.pi,
            plant_context=variables.plant_context,
        )
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_minimum_distance_lower_bound_constraint(self, variables):
        constraint = ik.MinimumDistanceLowerBoundConstraint(
            plant=variables.plant,
            bound=0.1,
            plant_context=variables.plant_context,
        )
        self.assertIsInstance(constraint, mp.Constraint)

        # Now set the new penalty function
        def penalty_fun(
            x: float, compute_grad: bool
        ) -> typing.Tuple[float, typing.Optional[float]]:
            if x < 0:
                if compute_grad:
                    return x**2, 2 * x
                else:
                    return x**2, None
            else:
                if compute_grad:
                    return 0.0, 0.0
                else:
                    return 0.0, None

        constraint = ik.MinimumDistanceLowerBoundConstraint(
            plant=variables.plant,
            bound=0.1,
            plant_context=variables.plant_context,
            penalty_function=penalty_fun,
            influence_distance_offset=3,
        )
        self.assertIsInstance(constraint, mp.Constraint)

        q = variables.plant.GetPositions(variables.plant_context)
        # Make sure that we can call Eval with the user defined penalty
        # function without riasing an exception.
        constraint.Eval(q)

        # Now test the case with penalty_function=None. It will use the
        # default penalty function.
        constraint = ik.MinimumDistanceLowerBoundConstraint(
            plant=variables.plant,
            bound=0.1,
            plant_context=variables.plant_context,
            penalty_function=None,
            influence_distance_offset=3,
        )
        self.assertIsInstance(constraint, mp.Constraint)
        # Make sure that we can call Eval with the user defined penalty
        # function without riasing an exception.
        constraint.Eval(q)

    @check_type_variables
    def test_minimum_distance_upper_bound_constraint(self, variables):
        constraint = ik.MinimumDistanceUpperBoundConstraint(
            plant=variables.plant,
            bound=0.1,
            influence_distance_offset=1,
            plant_context=variables.plant_context,
        )
        self.assertIsInstance(constraint, mp.Constraint)

        # Now set the new penalty function
        def penalty_fun(
            x: float, compute_grad: bool
        ) -> typing.Tuple[float, typing.Optional[float]]:
            if x < 0:
                if compute_grad:
                    return x**2, 2 * x
                else:
                    return x**2, None
            else:
                if compute_grad:
                    return 0.0, 0.0
                else:
                    return 0.0, None

        constraint = ik.MinimumDistanceUpperBoundConstraint(
            plant=variables.plant,
            bound=0.1,
            plant_context=variables.plant_context,
            influence_distance_offset=3,
            penalty_function=penalty_fun,
        )
        self.assertIsInstance(constraint, mp.Constraint)

        q = variables.plant.GetPositions(variables.plant_context)
        # Make sure that we can call Eval with the user defined penalty
        # function without riasing an exception.
        constraint.Eval(q)

        # Now test the case with penalty_function=None. It will use the
        # default penalty function.
        constraint = ik.MinimumDistanceUpperBoundConstraint(
            plant=variables.plant,
            bound=0.1,
            plant_context=variables.plant_context,
            influence_distance_offset=3,
            penalty_function=None,
        )
        self.assertIsInstance(constraint, mp.Constraint)
        # Make sure that we can call Eval with the user defined penalty
        # function without riasing an exception.
        constraint.Eval(q)

    def _make_robot_diagram(self):
        builder = RobotDiagramBuilder()
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

    def test_minimum_distance_constraints_with_collision_checker(self):
        # Test the MinimumDistance{Lower,Upper}BoundConstraint with
        # CollisionChecker.
        robot, index = self._make_robot_diagram()

        def distance_function(q1, q2):
            return np.linalg.norm(q1 - q2)

        collision_checker = SceneGraphCollisionChecker(
            model=robot,
            robot_model_instances=[index],
            configuration_distance_function=distance_function,
            edge_step_size=0.125,
        )
        collision_checker_context = (
            collision_checker.MakeStandaloneModelContext()
        )

        constraint_lower = ik.MinimumDistanceLowerBoundConstraint(
            collision_checker=collision_checker,
            bound=0.01,
            collision_checker_context=collision_checker_context,
            penalty_function=None,
            influence_distance_offset=0.1,
        )
        self.assertIsInstance(constraint_lower, mp.Constraint)

        constraint_upper = ik.MinimumDistanceUpperBoundConstraint(
            collision_checker=collision_checker,
            bound=0.1,
            collision_checker_context=collision_checker_context,
            penalty_function=None,
            influence_distance_offset=0.2,
        )
        self.assertIsInstance(constraint_upper, mp.Constraint)

    @check_type_variables
    def test_position_constraint(self, variables):
        constraint = ik.PositionConstraint(
            plant=variables.plant,
            frameA=variables.body1_frame,
            p_AQ_lower=[-0.1, -0.2, -0.3],
            p_AQ_upper=[-0.05, -0.12, -0.28],
            frameB=variables.body2_frame,
            p_BQ=[0.2, 0.3, 0.5],
            plant_context=variables.plant_context,
        )
        self.assertIsInstance(constraint, mp.Constraint)
        constraint.UpdateLowerBound(new_lb=np.array([-2, -3, -0.5]))
        constraint.UpdateUpperBound(new_ub=np.array([10.0, 0.5, 2.0]))
        constraint.set_bounds(new_lb=[-1, -2, -2.0], new_ub=[1.0, 2.0, 3.0])

        constraint = ik.PositionConstraint(
            plant=variables.plant,
            frameAbar=variables.body1_frame,
            X_AbarA=RigidTransform([-0.1, -0.2, -0.3]),
            p_AQ_lower=[-0.1, -0.2, -0.3],
            p_AQ_upper=[-0.05, -0.12, -0.28],
            frameB=variables.body2_frame,
            p_BQ=[0.2, 0.3, 0.5],
            plant_context=variables.plant_context,
        )
        self.assertIsInstance(constraint, mp.Constraint)
        constraint.UpdateLowerBound(new_lb=np.array([-2, -3, -0.5]))
        constraint.UpdateUpperBound(new_ub=np.array([10.0, 0.5, 2.0]))
        constraint.set_bounds(new_lb=[-1, -2, -2.0], new_ub=[1.0, 2.0, 3.0])

        # Construct without specifying p_BQ's value. p_BQ is a decision
        # variable.
        constraint = ik.PositionConstraint(
            plant=variables.plant,
            frameAbar=variables.body1_frame,
            X_AbarA=RigidTransform([-0.1, -0.2, -0.3]),
            p_AQ_lower=[-0.1, -0.2, -0.3],
            p_AQ_upper=[-0.05, -0.12, -0.28],
            frameB=variables.body2_frame,
            p_BQ=None,
            plant_context=variables.plant_context,
        )
        self.assertIsInstance(constraint, mp.Constraint)
        self.assertEqual(
            constraint.num_vars(), variables.plant.num_positions() + 3
        )

    @check_type_variables
    def test_position_cost(self, variables):
        cost = ik.PositionCost(
            plant=variables.plant,
            frameA=variables.body1_frame,
            p_AP=[-0.1, -0.2, -0.3],
            frameB=variables.body2_frame,
            p_BQ=[0.2, 0.3, 0.5],
            C=np.eye(3),
            plant_context=variables.plant_context,
        )
        self.assertIsInstance(cost, mp.Cost)

    @check_type_variables
    def test_com_position_constraint(self, variables):
        constraint = ik.ComPositionConstraint(
            plant=variables.plant,
            model_instances=None,
            expressed_frame=variables.plant.world_frame(),
            plant_context=variables.plant_context,
        )
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_com_in_polyhedron_constraint(self, variables):
        constraint = ik.ComInPolyhedronConstraint(
            plant=variables.plant,
            model_instances=None,
            expressed_frame=variables.plant.world_frame(),
            A=np.array([[1.0, 2.0, 3.0], [0.0, 1.0, 2.0]]),
            lb=np.array([0.1, 0.5]),
            ub=np.array([1.1, 1.5]),
            plant_context=variables.plant_context,
        )
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_orientation_constraint(self, variables):
        constraint = ik.OrientationConstraint(
            plant=variables.plant,
            frameAbar=variables.body1_frame,
            R_AbarA=RotationMatrix(),
            frameBbar=variables.body2_frame,
            R_BbarB=RotationMatrix(),
            theta_bound=0.2 * math.pi,
            plant_context=variables.plant_context,
        )
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_orientation_cost(self, variables):
        cost = ik.OrientationCost(
            plant=variables.plant,
            frameAbar=variables.body1_frame,
            R_AbarA=RotationMatrix(),
            frameBbar=variables.body2_frame,
            R_BbarB=RotationMatrix(),
            c=1.0,
            plant_context=variables.plant_context,
        )
        self.assertIsInstance(cost, mp.Cost)

    @check_type_variables
    def test_point_to_point_distance_constraint(self, variables):
        constraint = ik.PointToPointDistanceConstraint(
            plant=variables.plant,
            frame1=variables.body1_frame,
            p_B1P1=[0.1, 0.2, 0.3],
            frame2=variables.body2_frame,
            p_B2P2=[0.3, 0.4, 0.5],
            distance_lower=0.1,
            distance_upper=0.2,
            plant_context=variables.plant_context,
        )
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_point_to_line_distance_constraint(self, variables):
        constraint = ik.PointToLineDistanceConstraint(
            plant=variables.plant,
            frame_point=variables.body1_frame,
            p_B1P=[0.1, 0.2, 0.3],
            frame_line=variables.body2_frame,
            p_B2Q=[0.3, 0.4, 0.5],
            n_B2=[0.2, 0.3, 0.4],
            distance_lower=0.1,
            distance_upper=0.2,
            plant_context=variables.plant_context,
        )
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_polyhedron_constraint(self, variables):
        constraint = ik.PolyhedronConstraint(
            plant=variables.plant,
            frameF=variables.body1_frame,
            frameG=variables.body2_frame,
            p_GP=np.array([[0.2, 0.3], [0.1, 0.5], [1.2, 1.3]]),
            A=np.array([[1, 2, 3, 4, 5, 6]]),
            b=np.array([10.0]),
            plant_context=variables.plant_context,
        )
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_distance_constraint_2(self, variables):
        inspector = self.scene_graph_f.model_inspector()
        frame_id1 = inspector.GetGeometryIdByName(
            self.plant_f.GetBodyFrameIdOrThrow(
                self.plant_f.GetBodyByName("body1").index()
            ),
            pydrake.geometry.Role.kProximity,
            "two_bodies::body1_collision",
        )
        frame_id2 = inspector.GetGeometryIdByName(
            self.plant_f.GetBodyFrameIdOrThrow(
                self.plant_f.GetBodyByName("body2").index()
            ),
            pydrake.geometry.Role.kProximity,
            "two_bodies::body2_collision",
        )
        constraint = ik.DistanceConstraint(
            plant=variables.plant,
            geometry_pair=(frame_id1, frame_id2),
            distance_lower=0.1,
            distance_upper=0.2,
            plant_context=variables.plant_context,
        )
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_unit_quaternion_constraint(self, variables):
        constraint = ik.UnitQuaternionConstraint()
        self.assertIsInstance(constraint, mp.Constraint)


class TestGlobalInverseKinematics(unittest.TestCase):
    def test_options(self):
        options = ik.GlobalInverseKinematics.Options()
        self.assertEqual(
            repr(options),
            "".join(
                [
                    "GlobalInverseKinematics.Options(",
                    "num_intervals_per_half_axis=2, ",
                    "approach=Approach.kBilinearMcCormick, "
                    "interval_binning=IntervalBinning.kLogarithmic, "
                    "linear_constraint_only=False)",
                ]
            ),
        )
        self.assertEqual(options.num_intervals_per_half_axis, 2)
        mircg = mp.MixedIntegerRotationConstraintGenerator
        self.assertEqual(options.approach, mircg.Approach.kBilinearMcCormick)
        self.assertEqual(
            options.interval_binning, mp.IntervalBinning.kLogarithmic
        )
        self.assertFalse(options.linear_constraint_only)

    def test_polytope3d(self):
        A = np.eye(3)
        b = np.zeros(3)
        p = ik.GlobalInverseKinematics.Polytope3D(A, b)
        np.testing.assert_array_equal(p.A, A)
        np.testing.assert_array_equal(p.b, b)
        self.assertEqual(
            repr(p),
            textwrap.dedent("""
            GlobalInverseKinematics.Polytope(A=[[1. 0. 0.]
             [0. 1. 0.]
             [0. 0. 1.]], b=[0. 0. 0.])""").strip(),
        )

    def test_api(self):
        plant = MultibodyPlant(time_step=0.01)
        (model_instance,) = Parser(plant).AddModels(
            FindResourceOrThrow(
                "drake/bindings/pydrake/multibody/test/two_bodies.sdf"
            )
        )
        plant.Finalize()
        context = plant.CreateDefaultContext()
        options = ik.GlobalInverseKinematics.Options()
        global_ik = ik.GlobalInverseKinematics(plant=plant, options=options)
        self.assertIsInstance(global_ik.prog(), mp.MathematicalProgram)
        self.assertIsInstance(
            global_ik.get_mutable_prog(), mp.MathematicalProgram
        )
        body_index_A = plant.GetBodyIndices(model_instance)[0]
        body_index_B = plant.GetBodyIndices(model_instance)[1]
        self.assertEqual(
            global_ik.body_rotation_matrix(body_index=body_index_A).shape,
            (3, 3),
        )
        self.assertEqual(
            global_ik.body_position(body_index=body_index_A).shape, (3,)
        )
        global_ik.AddWorldPositionConstraint(
            body_index=body_index_A,
            p_BQ=[0, 0, 0],
            box_lb_F=[-np.inf, -np.inf, -np.inf],
            box_ub_F=[np.inf, np.inf, np.inf],
            X_WF=RigidTransform(),
        )
        global_ik.AddWorldRelativePositionConstraint(
            body_index_B=body_index_B,
            p_BQ=[0, 0, 0],
            body_index_A=body_index_A,
            p_AP=[0, 0, 0],
            box_lb_F=[-np.inf, -np.inf, -np.inf],
            box_ub_F=[np.inf, np.inf, np.inf],
            X_WF=RigidTransform(),
        )
        global_ik.AddWorldOrientationConstraint(
            body_index=body_index_A,
            desired_orientation=Quaternion(),
            angle_tol=np.inf,
        )
        global_ik.AddPostureCost(
            q_desired=plant.GetPositions(context),
            body_position_cost=[1] * plant.num_bodies(),
            body_orientation_cost=[1] * plant.num_bodies(),
        )

        regions_vertices = (
            np.array(
                [
                    [
                        [-1, -1, -1],
                        [-1, -1, 1],
                        [-1, 1, -1],
                        [-1, 1, 1],
                        [1, -1, -1],
                        [1, -1, 1],
                        [1, 1, -1],
                        [1, 1, 1],
                    ]
                ],
                dtype=float,
            ).reshape(1, 3, 8)
            * 10
        )
        vars1 = global_ik.BodyPointInOneOfRegions(
            body_index=body_index_A,
            p_BQ=np.zeros(3),
            region_vertices=regions_vertices,
        )
        self.assertIsInstance(vars1, np.ndarray)
        self.assertTrue(len(vars1) == 1)
        self.assertIsInstance(vars1[0], Variable)

        polytope_A = np.array([[1.0, 0.0, 0.0]])
        polytope_b = np.array([10.0])
        regions_polytope3d = [
            ik.GlobalInverseKinematics.Polytope3D(A=polytope_A, b=polytope_b)
        ]
        np.testing.assert_array_equal(regions_polytope3d[0].A, polytope_A)
        np.testing.assert_array_equal(regions_polytope3d[0].b, polytope_b)
        vars2 = global_ik.BodySphereInOneOfPolytopes(
            body_index=body_index_A,
            p_BQ=np.zeros(3),
            radius=0.1,
            polytopes=regions_polytope3d,
        )
        self.assertIsInstance(vars2, np.ndarray)
        self.assertTrue(len(vars2) == 1)
        self.assertIsInstance(vars2[0], Variable)

        gurobi_solver = mp.GurobiSolver()
        if gurobi_solver.available():
            global_ik.SetInitialGuess(q=plant.GetPositions(context))
            result = gurobi_solver.Solve(global_ik.prog())
            self.assertTrue(result.is_success())
            global_ik.ReconstructGeneralizedPositionSolution(result=result)

    def test_joint_limits(self):
        # We need a separate test since the AddJointLimitsConstraint method
        # is not usable with floating bodies.
        plant = MultibodyPlant(time_step=0.01)
        (model_instance,) = Parser(plant).AddModels(
            FindResourceOrThrow(
                "drake/bindings/pydrake/multibody/test/double_pendulum.sdf"
            )
        )
        plant.Finalize()
        options = ik.GlobalInverseKinematics.Options()
        global_ik = ik.GlobalInverseKinematics(plant=plant, options=options)

        body_index = plant.GetBodyIndices(model_instance)[1]
        global_ik.AddJointLimitConstraint(
            body_index=body_index,
            joint_lower_bound=-10.0,
            joint_upper_bound=10.0,
            linear_constraint_approximation=False,
        )
