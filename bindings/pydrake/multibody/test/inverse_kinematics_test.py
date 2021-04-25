from pydrake.multibody import inverse_kinematics as ik

from collections import namedtuple
from functools import partial, wraps
import math
import unittest

import numpy as np

import pydrake
from pydrake.common import FindResourceOrThrow
from pydrake.common.eigen_geometry import Quaternion
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    MultibodyPlant, AddMultibodyPlantSceneGraph)
import pydrake.solvers.mathematicalprogram as mp
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
            builder, MultibodyPlant(time_step=0.01))
        Parser(self.plant).AddModelFromFile(FindResourceOrThrow(
                "drake/bindings/pydrake/multibody/test/two_bodies.sdf"))
        self.plant.Finalize()
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        plant_context = diagram.GetMutableSubsystemContext(
            self.plant, diagram_context)
        self.body1_frame = self.plant.GetBodyByName("body1").body_frame()
        self.body2_frame = self.plant.GetBodyByName("body2").body_frame()
        self.ik_two_bodies = ik.InverseKinematics(
            plant=self.plant, plant_context=plant_context)
        # Test non-SceneGraph constructor.
        ik.InverseKinematics(plant=self.plant)
        self.prog = self.ik_two_bodies.get_mutable_prog()
        self.q = self.ik_two_bodies.q()

        # Test constructor without joint limits
        ik.InverseKinematics(plant=self.plant, with_joint_limits=False)
        ik.InverseKinematics(
            plant=self.plant, plant_context=plant_context,
            with_joint_limits=False)

        def squaredNorm(x):
            return np.array([x[0] ** 2 + x[1] ** 2 + x[2] ** 2 + x[3] ** 2])

        self.prog.AddConstraint(
            squaredNorm, [1], [1], self._body1_quat(self.q))
        self.prog.AddConstraint(
            squaredNorm, [1], [1], self._body2_quat(self.q))
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

    def test_AddPositionConstraint(self):
        p_BQ = np.array([0.2, 0.3, 0.5])
        p_AQ_lower = np.array([-0.1, -0.2, -0.3])
        p_AQ_upper = np.array([-0.05, -0.12, -0.28])

        self.ik_two_bodies.AddPositionConstraint(
            frameB=self.body1_frame, p_BQ=p_BQ,
            frameA=self.body2_frame,
            p_AQ_lower=p_AQ_lower, p_AQ_upper=p_AQ_upper)
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
            body1_rotmat.dot(p_BQ) + body1_pos - body2_pos)
        self.assertTrue(np.greater(
            p_AQ, p_AQ_lower - 1E-6 * np.ones((3, 1))).all())
        self.assertTrue(np.less(
            p_AQ, p_AQ_upper + 1E-6 * np.ones((3, 1))).all())

        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        self.assertTrue(np.allclose(result.GetSolution(self.q), q_val))
        np.testing.assert_array_equal(self.plant.GetPositions(
            self.ik_two_bodies.context()), q_val)
        self.assertIs(self.ik_two_bodies.get_mutable_context(),
                      self.ik_two_bodies.context())

    def test_AddOrientationConstraint(self):
        theta_bound = 0.2 * math.pi
        R_AbarA = RotationMatrix(quaternion=Quaternion(0.5, -0.5, 0.5, 0.5))
        R_BbarB = RotationMatrix(
            quaternion=Quaternion(1.0 / 3, 2.0 / 3, 0, 2.0 / 3))
        self.ik_two_bodies.AddOrientationConstraint(
            frameAbar=self.body1_frame, R_AbarA=R_AbarA,
            frameBbar=self.body2_frame, R_BbarB=R_BbarB,
            theta_bound=theta_bound)

        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        q_val = result.GetSolution(self.q)

        body1_quat = self._body1_quat(q_val)
        body2_quat = self._body2_quat(q_val)
        body1_rotmat = Quaternion(body1_quat).rotation()
        body2_rotmat = Quaternion(body2_quat).rotation()
        R_AbarBbar = body1_rotmat.transpose().dot(body2_rotmat)
        R_AB = R_AbarA.matrix().transpose().dot(
            R_AbarBbar.dot(R_BbarB.matrix()))
        self.assertGreater(R_AB.trace(), 1 + 2 * math.cos(theta_bound) - 1E-6)

        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        q_val = result.GetSolution(self.q)

        body1_quat = self._body1_quat(q_val)
        body2_quat = self._body2_quat(q_val)
        body1_rotmat = Quaternion(body1_quat).rotation()
        body2_rotmat = Quaternion(body2_quat).rotation()
        R_AbarBbar = body1_rotmat.transpose().dot(body2_rotmat)
        R_AB = R_AbarA.matrix().transpose().dot(
            R_AbarBbar.dot(R_BbarB.matrix()))
        self.assertGreater(R_AB.trace(),
                           1 + 2 * math.cos(theta_bound) - 1E-6)

    def test_AddGazeTargetConstraint(self):
        p_AS = np.array([0.1, 0.2, 0.3])
        n_A = np.array([0.3, 0.5, 1.2])
        p_BT = np.array([1.1, 0.2, 1.5])
        cone_half_angle = 0.2 * math.pi

        self.ik_two_bodies.AddGazeTargetConstraint(
            frameA=self.body1_frame, p_AS=p_AS, n_A=n_A,
            frameB=self.body2_frame, p_BT=p_BT,
            cone_half_angle=cone_half_angle)

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
        self.assertGreater(p_ST_W.dot(n_W), np.linalg.norm(
            p_ST_W) * np.linalg.norm(n_W) * math.cos(cone_half_angle) - 1E-6)

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
            (np.linalg.norm(p_ST_W)
                * np.linalg.norm(n_W)
                * math.cos(cone_half_angle))
            - 1E-6)

    def test_AddAngleBetweenVectorsConstraint(self):
        na_A = np.array([0.2, -0.4, 0.9])
        nb_B = np.array([1.4, -0.1, 1.8])

        angle_lower = 0.2 * math.pi
        angle_upper = 0.2 * math.pi

        self.ik_two_bodies.AddAngleBetweenVectorsConstraint(
            frameA=self.body1_frame, na_A=na_A,
            frameB=self.body2_frame, nb_B=nb_B,
            angle_lower=angle_lower, angle_upper=angle_upper)
        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())

        q_val = result.GetSolution(self.q)
        body1_quat = self._body1_quat(q_val)
        body2_quat = self._body2_quat(q_val)
        body1_rotmat = Quaternion(body1_quat).rotation()
        body2_rotmat = Quaternion(body2_quat).rotation()

        na_W = body1_rotmat.dot(na_A)
        nb_W = body2_rotmat.dot(nb_B)

        angle = math.acos(na_W.transpose().dot(nb_W)
                          / (np.linalg.norm(na_W) * np.linalg.norm(nb_W)))

        self.assertLess(math.fabs(angle - angle_lower), 1E-6)

        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        self.assertTrue(np.allclose(result.GetSolution(self.q), q_val))

    def test_AddPointToPointDistanceConstraint(self):
        p_B1P1 = np.array([0.2, -0.4, 0.9])
        p_B2P2 = np.array([1.4, -0.1, 1.8])

        distance_lower = 0.1
        distance_upper = 0.2

        self.ik_two_bodies.AddPointToPointDistanceConstraint(
            frame1=self.body1_frame, p_B1P1=p_B1P1,
            frame2=self.body2_frame, p_B2P2=p_B2P2,
            distance_lower=distance_lower, distance_upper=distance_upper)
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

    def test_AddMinimumDistanceConstraint(self):
        ik = self.ik_two_bodies
        W = self.plant.world_frame()
        B1 = self.body1_frame
        B2 = self.body2_frame

        min_distance = 0.1
        tol = 1e-2
        radius1 = 0.1
        radius2 = 0.2

        ik.AddMinimumDistanceConstraint(minimum_distance=min_distance)
        context = self.plant.CreateDefaultContext()
        self.plant.SetFreeBodyPose(
            context, B1.body(), RigidTransform([0, 0, 0.01]))
        self.plant.SetFreeBodyPose(
            context, B2.body(), RigidTransform([0, 0, -0.01]))

        def get_min_distance_actual():
            X = partial(self.plant.CalcRelativeTransform, context)
            distance = np.linalg.norm(
                X(W, B1).translation() - X(W, B2).translation())
            return distance - radius1 - radius2

        self.assertLess(get_min_distance_actual(), min_distance - tol)
        self.prog.SetInitialGuess(ik.q(), self.plant.GetPositions(context))
        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        q_val = result.GetSolution(ik.q())
        self.plant.SetPositions(context, q_val)
        self.assertGreater(get_min_distance_actual(), min_distance - tol)

        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        self.assertTrue(np.allclose(result.GetSolution(ik.q()), q_val))

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
                self.plant.GetBodyByName("body1").index()),
            pydrake.geometry.Role.kProximity, "two_bodies::body1_collision")
        frame_id2 = inspector.GetGeometryIdByName(
            self.plant.GetBodyFrameIdOrThrow(
                self.plant.GetBodyByName("body2").index()),
            pydrake.geometry.Role.kProximity, "two_bodies::body2_collision")
        ik.AddDistanceConstraint(
            geometry_pair=(frame_id1, frame_id2),
            distance_lower=distance_lower, distance_upper=distance_upper)

        context = self.plant.CreateDefaultContext()
        self.plant.SetFreeBodyPose(
            context, B1.body(), RigidTransform([0, 0, 0.01]))
        self.plant.SetFreeBodyPose(
            context, B2.body(), RigidTransform([0, 0, -0.01]))

        def get_min_distance_actual():
            X = partial(self.plant.CalcRelativeTransform, context)
            distance = np.linalg.norm(
                X(W, B1).translation() - X(W, B2).translation())
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
            builder_f, MultibodyPlant(time_step=0.01))
        Parser(self.plant_f).AddModelFromFile(FindResourceOrThrow(
                "drake/bindings/pydrake/multibody/test/two_bodies.sdf"))
        self.plant_f.Finalize()
        diagram_f = builder_f.Build()
        diagram_ad = diagram_f.ToAutoDiffXd()
        plant_ad = diagram_ad.GetSubsystemByName(self.plant_f.get_name())

        TypeVariables = namedtuple(
            "TypeVariables",
            ("plant", "plant_context", "body1_frame", "body2_frame"))

        def make_type_variables(plant_T, diagram_T):
            diagram_context_T = diagram_T.CreateDefaultContext()
            return TypeVariables(
                plant=plant_T,
                plant_context=diagram_T.GetMutableSubsystemContext(
                    plant_T, diagram_context_T),
                body1_frame=plant_T.GetBodyByName("body1").body_frame(),
                body2_frame=plant_T.GetBodyByName("body2").body_frame())

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
            plant_context=variables.plant_context)
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_distance_constraint(self, variables):

        def get_sphere_geometry_id(frame):
            id_, = variables.plant.GetCollisionGeometriesForBody(frame.body())
            return id_

        constraint = ik.DistanceConstraint(
            plant=variables.plant,
            geometry_pair=(
                get_sphere_geometry_id(variables.body1_frame),
                get_sphere_geometry_id(variables.body2_frame)),
            plant_context=variables.plant_context,
            distance_lower=0.1,
            distance_upper=2)
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_gaze_target_constraint(self, variables):
        constraint = ik.GazeTargetConstraint(
            plant=variables.plant,
            frameA=variables.body1_frame,
            p_AS=[0.1, 0.2, 0.3], n_A=[0.3, 0.5, 1.2],
            frameB=variables.body2_frame, p_BT=[1.1, 0.2, 1.5],
            cone_half_angle=0.2 * math.pi,
            plant_context=variables.plant_context)
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_minimum_distance_constraint(self, variables):
        constraint = ik.MinimumDistanceConstraint(
            plant=variables.plant,
            minimum_distance=0.1,
            plant_context=variables.plant_context)
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_position_constraint(self, variables):
        constraint = ik.PositionConstraint(
            plant=variables.plant,
            frameA=variables.body1_frame,
            p_AQ_lower=[-0.1, -0.2, -0.3],
            p_AQ_upper=[-0.05, -0.12, -0.28],
            frameB=variables.body2_frame,
            p_BQ=[0.2, 0.3, 0.5], plant_context=variables.plant_context)
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_com_position_constraint(self, variables):
        constraint = ik.ComPositionConstraint(
            plant=variables.plant,
            model_instances=None,
            expressed_frame=variables.plant.world_frame(),
            plant_context=variables.plant_context)
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_com_in_polyhedron_constraint(self, variables):
        constraint = ik.ComInPolyhedronConstraint(
            plant=variables.plant,
            model_instances=None,
            expressed_frame=variables.plant.world_frame(),
            A=np.array([[1., 2., 3.], [0., 1., 2.]]),
            lb=np.array([0.1, 0.5]),
            ub=np.array([1.1, 1.5]),
            plant_context=variables.plant_context)
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_orientation_constraint(self, variables):
        constraint = ik.OrientationConstraint(
            plant=variables.plant,
            frameAbar=variables.body1_frame, R_AbarA=RotationMatrix(),
            frameBbar=variables.body2_frame, R_BbarB=RotationMatrix(),
            theta_bound=0.2 * math.pi, plant_context=variables.plant_context)
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_point_to_point_distance_constraint(self, variables):
        constraint = ik.PointToPointDistanceConstraint(
            plant=variables.plant,
            frame1=variables.body1_frame, p_B1P1=[0.1, 0.2, 0.3],
            frame2=variables.body2_frame, p_B2P2=[0.3, 0.4, 0.5],
            distance_lower=0.1, distance_upper=0.2,
            plant_context=variables.plant_context)
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_distance_constraint(self, variables):
        inspector = self.scene_graph_f.model_inspector()
        frame_id1 = inspector.GetGeometryIdByName(
            self.plant_f.GetBodyFrameIdOrThrow(
                self.plant_f.GetBodyByName("body1").index()),
            pydrake.geometry.Role.kProximity, "two_bodies::body1_collision")
        frame_id2 = inspector.GetGeometryIdByName(
            self.plant_f.GetBodyFrameIdOrThrow(
                self.plant_f.GetBodyByName("body2").index()),
            pydrake.geometry.Role.kProximity, "two_bodies::body2_collision")
        constraint = ik.DistanceConstraint(
            plant=variables.plant, geometry_pair=(frame_id1, frame_id2),
            distance_lower=0.1, distance_upper=0.2,
            plant_context=variables.plant_context)
        self.assertIsInstance(constraint, mp.Constraint)

    @check_type_variables
    def test_unit_quaternion_constraint(self, variables):
        constraint = ik.UnitQuaternionConstraint()
        self.assertIsInstance(constraint, mp.Constraint)
