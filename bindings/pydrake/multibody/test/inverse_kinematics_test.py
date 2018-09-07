from pydrake.multibody import inverse_kinematics as ik

import math

from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.multibody_tree.parsing import (
    AddModelFromSdfFile,
)
from pydrake.multibody.benchmarks.acrobot import (
    MakeAcrobotPlant,
)
import unittest
import pydrake.solvers.mathematicalprogram as mp
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.util.eigen_geometry import Quaternion


class TestInverseKinematics(unittest.TestCase):
    def setUp(self):
        file_name = FindResourceOrThrow(
            "drake/bindings/pydrake/multibody/test/two_bodies.sdf")
        self.plant = MultibodyPlant(time_step=0.01)
        model_instance = AddModelFromSdfFile(
            file_name=file_name, plant=self.plant, scene_graph=None)
        self.plant.Finalize()
        self.link1_frame = self.plant.GetBodyByName("body1").body_frame()
        self.link2_frame = self.plant.GetBodyByName("body2").body_frame()
        self.ik_two_bodies = ik.InverseKinematics(self.plant)
        self.prog = self.ik_two_bodies.get_mutable_prog()
        self.q = self.ik_two_bodies.q()

        def squaredNorm(x):
            return np.array([x[0] ** 2 + x[1] ** 2 + x[2] ** 2 + x[3] ** 2])

        self.prog.AddConstraint(squaredNorm, [1], [1], self.q[0:4])
        self.prog.AddConstraint(squaredNorm, [1], [1], self.q[7:11])
        self.prog.SetInitialGuess(self.q[0:4], [1, 0, 0, 0])
        self.prog.SetInitialGuess(self.q[7:11], [1, 0, 0, 0])

    def test_AddPositionConstraint(self):
        p_BQ = np.array([0.2, 0.3, 0.5])
        p_AQ_lower = np.array([-0.1, -0.2, -0.3])
        p_AQ_upper = np.array([-0.05, -0.12, -0.28])

        self.ik_two_bodies.AddPositionConstraint(
            self.link1_frame, p_BQ, self.link2_frame, p_AQ_lower, p_AQ_upper)
        result = self.prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)
        q_val = self.prog.GetSolution(self.q)

        body1_quat = q_val[0:4]
        body1_pos = q_val[4:7]
        body2_quat = q_val[7:11]
        body2_pos = q_val[11:14]
        body1_rotmat = Quaternion(body1_quat).rotation()
        body2_rotmat = Quaternion(body2_quat).rotation()
        p_AQ = body2_rotmat.transpose().dot(
            body1_rotmat.dot(p_BQ) + body1_pos - body2_pos)
        self.assertTrue(np.greater(p_AQ, p_AQ_lower -
                                   1E-6 * np.ones((3, 1))).all())
        self.assertTrue(np.less(p_AQ, p_AQ_upper +
                                1E-6 * np.ones((3, 1))).all())

    def test_AddOrientationConstraint(self):
        theta_bound = 0.2 * math.pi
        self.ik_two_bodies.AddOrientationConstraint(
            frameA=self.link1_frame, frameB=self.link2_frame,
            theta_bound=theta_bound)
        result = self.prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)

        q_val = self.prog.GetSolution(self.q)

        body1_quat = q_val[0:4]
        body2_quat = q_val[7:11]
        body1_rotmat = Quaternion(body1_quat).rotation()
        body2_rotmat = Quaternion(body2_quat).rotation()
        R_AB = body1_rotmat.transpose().dot(body2_rotmat)
        self.assertGreater(R_AB.trace(), 1 + 2 * math.cos(theta_bound) - 1E-6)

    def test_AddGazeTargetConstraint(self):
        p_AS = np.array([0.1, 0.2, 0.3])
        n_A = np.array([0.3, 0.5, 1.2])
        p_BT = np.array([1.1, 0.2, 1.5])
        cone_half_angle = 0.2 * math.pi

        self.ik_two_bodies.AddGazeTargetConstraint(
            frameA=self.link1_frame, p_AS=p_AS, n_A=n_A,
            frameB=self.link2_frame, p_BT=p_BT,
            cone_half_angle=cone_half_angle)
        result = self.prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)

        q_val = self.prog.GetSolution(self.q)
        body1_quat = q_val[0:4]
        body1_pos = q_val[4:7]
        body2_quat = q_val[7:11]
        body2_pos = q_val[11:14]
        body1_rotmat = Quaternion(body1_quat).rotation()
        body2_rotmat = Quaternion(body2_quat).rotation()

        p_WS = body1_pos + body1_rotmat.dot(p_AS)
        p_WT = body2_pos + body2_rotmat.dot(p_BT)
        p_ST_W = p_WT - p_WS
        n_W = body1_rotmat.dot(n_A)
        self.assertGreater(p_ST_W.dot(n_W), np.linalg.norm(
            p_ST_W) * np.linalg.norm(n_W) * math.cos(cone_half_angle) - 1E-6)

    def test_AddAngleBetweenVectorsConstraint(self):
        na_A = np.array([0.2, -0.4, 0.9])
        nb_B = np.array([1.4, -0.1, 1.8])

        angle_lower = 0.2 * math.pi
        angle_upper = 0.2 * math.pi

        self.ik_two_bodies.AddAngleBetweenVectorsConstraint(
            frameA=self.link1_frame, na_A=na_A,
            frameB=self.link2_frame, nb_B=nb_B,
            angle_lower=angle_lower, angle_upper=angle_upper)
        result = self.prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)

        q_val = self.prog.GetSolution(self.q)
        body1_quat = q_val[0:4]
        body2_quat = q_val[7:11]
        body1_rotmat = Quaternion(body1_quat).rotation()
        body2_rotmat = Quaternion(body2_quat).rotation()

        na_W = body1_rotmat.dot(na_A)
        nb_W = body2_rotmat.dot(nb_B)

        angle = math.acos(na_W.transpose().dot(nb_W) /
                          (np.linalg.norm(na_W) * np.linalg.norm(nb_W)))

        self.assertLess(math.fabs(angle - angle_lower), 1E-6)
