from pydrake.multibody import inverse_kinematics as ik

from functools import partial
import math
import unittest
import warnings

import numpy as np
from numpy.linalg import norm

from pydrake.common import FindResourceOrThrow
from pydrake.common.eigen_geometry import Quaternion
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.plant import (
    MultibodyPlant, AddMultibodyPlantSceneGraph)
from pydrake.multibody.parsing import Parser
from pydrake.multibody.benchmarks.acrobot import (
    MakeAcrobotPlant,
)
import pydrake.solvers.mathematicalprogram as mp
from pydrake.systems.framework import DiagramBuilder

# TODO(eric.cousineau): Replace manual coordinate indexing with more semantic
# operations (`CalcRelativeTransform`, `SetFreeBodyPose`).


class TestInverseKinematics(unittest.TestCase):
    """
    This test reflects inverse_kinematics_test.cc
    """
    def setUp(self):
        file_name = FindResourceOrThrow(
            "drake/bindings/pydrake/multibody/test/two_bodies.sdf")
        builder = DiagramBuilder()
        self.plant, _ = AddMultibodyPlantSceneGraph(
            builder, MultibodyPlant(time_step=0.01))
        model_instance = Parser(self.plant).AddModelFromFile(file_name)
        self.plant.Finalize()
        self.diagram = builder.Build()
        self.diagram_context = self.diagram.CreateDefaultContext()
        self.plant_context = self.diagram.GetMutableSubsystemContext(
            self.plant, self.diagram_context)
        self.body1_frame = self.plant.GetBodyByName("body1").body_frame()
        self.body2_frame = self.plant.GetBodyByName("body2").body_frame()
        self.ik_two_bodies = ik.InverseKinematics(
            plant=self.plant, plant_context=self.plant_context)
        # Test non-SceneGraph constructor.
        ik.InverseKinematics(plant=self.plant)
        self.prog = self.ik_two_bodies.get_mutable_prog()
        self.q = self.ik_two_bodies.q()

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
        self.assertTrue(np.greater(p_AQ, p_AQ_lower -
                                   1E-6 * np.ones((3, 1))).all())
        self.assertTrue(np.less(p_AQ, p_AQ_upper +
                                1E-6 * np.ones((3, 1))).all())

        with catch_drake_warnings(expected_count=2):
            self.assertEqual(
                self.prog.Solve(), mp.SolutionResult.kSolutionFound)
            self.assertTrue(np.allclose(
                self.prog.GetSolution(self.q), q_val))

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

        with catch_drake_warnings(expected_count=2):
            self.assertEqual(
                self.prog.Solve(), mp.SolutionResult.kSolutionFound)
            q_val = self.prog.GetSolution(self.q)

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

        with catch_drake_warnings(expected_count=2):
            self.assertEqual(
                self.prog.Solve(), mp.SolutionResult.kSolutionFound)
            q_val = self.prog.GetSolution(self.q)

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
                p_ST_W) * np.linalg.norm(n_W) *
                math.cos(cone_half_angle) - 1E-6)

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

        angle = math.acos(na_W.transpose().dot(nb_W) /
                          (np.linalg.norm(na_W) * np.linalg.norm(nb_W)))

        self.assertLess(math.fabs(angle - angle_lower), 1E-6)

        with catch_drake_warnings(expected_count=2):
            self.assertEqual(
                self.prog.Solve(), mp.SolutionResult.kSolutionFound)
            self.assertTrue(np.allclose(
                self.prog.GetSolution(self.q), q_val))

    def test_AddMinimumDistanceConstraint(self):
        ik = self.ik_two_bodies
        W = self.plant.world_frame()
        B1 = self.body1_frame
        B2 = self.body2_frame

        min_distance = 0.1
        tol = 1e-2
        radius1 = 0.1
        radius2 = 0.2

        ik.AddMinimumDistanceConstraint(minimal_distance=min_distance)
        context = self.plant.CreateDefaultContext()
        self.plant.SetFreeBodyPose(
            context, B1.body(), RigidTransform([0, 0, 0.01]))
        self.plant.SetFreeBodyPose(
            context, B2.body(), RigidTransform([0, 0, -0.01]))

        def get_min_distance_actual():
            X = partial(self.plant.CalcRelativeTransform, context)
            distance = norm(X(W, B1).translation() - X(W, B2).translation())
            return distance - radius1 - radius2

        self.assertLess(get_min_distance_actual(), min_distance - tol)
        self.prog.SetInitialGuess(ik.q(), self.plant.GetPositions(context))
        result = mp.Solve(self.prog)
        self.assertTrue(result.is_success())
        q_val = result.GetSolution(ik.q())
        self.plant.SetPositions(context, q_val)
        self.assertGreater(get_min_distance_actual(), min_distance - tol)

        with catch_drake_warnings(expected_count=2):
            self.assertEqual(
                self.prog.Solve(), mp.SolutionResult.kSolutionFound)
            self.assertTrue(np.allclose(
                self.prog.GetSolution(ik.q()), q_val))
