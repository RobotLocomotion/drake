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


def quaternionToRotationMatrix(q):
    ww = q[0] * q[0]
    wx = q[0] * q[1]
    wy = q[0] * q[2]
    wz = q[0] * q[3]
    xx = q[1] * q[1]
    xy = q[1] * q[2]
    xz = q[1] * q[3]
    yy = q[2] * q[2]
    yz = q[2] * q[3]
    zz = q[3] * q[3]
    return np.array([[ww + xx - yy - zz, 2 * (xy - wz), 2 * (xz + wy)],
                     [2 * (xy + wz), ww + yy - xx - zz, 2 * (yz - wx)],
                     [2 * (xz - wy), 2 * (yz + wx), ww + zz - xx - yy]])


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
        result = self.ik_two_bodies.get_mutable_prog().Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)
        q_val = self.prog.GetSolution(self.q)

        body1_quat = q_val[0:4]
        body1_pos = q_val[4:7]
        body2_quat = q_val[7:11]
        body2_pos = q_val[11:14]
        body1_rotmat = quaternionToRotationMatrix(body1_quat)
        body2_rotmat = quaternionToRotationMatrix(body2_quat)
        p_AQ = body2_rotmat.transpose().dot(
            body1_rotmat.dot(p_BQ) + body1_pos - body2_pos)
        self.assertTrue(np.greater(p_AQ, p_AQ_lower -
                                   1E-6 * np.ones((3, 1))).all())
        self.assertTrue(np.less(p_AQ, p_AQ_upper +
                                1E-6 * np.ones((3, 1))).all())

    def test_AddOrientationConstraint(self):
        angle_bound = 0.2 * math.pi
        self.ik_two_bodies.AddOrientationConstraint(
            self.link1_frame, self.link2_frame, angle_bound)
        result = self.ik_two_bodies.get_mutable_prog().Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)

        q_val = self.prog.GetSolution(self.q)

        body1_quat = q_val[0:4]
        body2_quat = q_val[7:11]
        body1_rotmat = quaternionToRotationMatrix(body1_quat)
        body2_rotmat = quaternionToRotationMatrix(body2_quat)
        R_AB = body1_rotmat.transpose().dot(body2_rotmat)
        self.assertGreater(R_AB.trace(), 1 + 2 * math.cos(angle_bound) - 1E-6)

    def test_AddGazeTargetConstraint(self):
        p_AS = np.array([0.1, 0.2, 0.3])
        n_A = np.array([0.3, 0.5, 1.2])
        p_BT = np.array([1.1, 0.2, 1.5])
        cone_half_angle = 0.2 * math.pi

        self.ik_two_bodies.AddGazeTargetConstraint(
            self.link1_frame, p_AS, n_A, self.link2_frame, p_BT, cone_half_angle)
        result = self.ik_two_bodies.get_mutable_prog().Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)

        q_val = self.prog.GetSolution(self.q)
        body1_quat = q_val[0:4]
        body1_pos = q_val[4:7]
        body2_quat = q_val[7:11]
        body2_pos = q_val[11:14]
        body1_rotmat = quaternionToRotationMatrix(body1_quat)
        body2_rotmat = quaternionToRotationMatrix(body2_quat)

        p_WS = body1_pos + body1_rotmat.dot(p_AS)
        p_WT = body2_pos + body2_rotmat.dot(p_BT)
        p_ST_W = p_WT - p_WS
        n_A_W = body1_rotmat.dot(n_A)
        self.assertGreater(p_ST_W.dot(n_A_W), np.linalg.norm(
            p_ST_W) * np.linalg.norm(n_A_W) * math.cos(cone_half_angle) - 1E-6)

    def test_AddAngleBetweenVectorsConstraint(self):
        n_A = np.array([0.2, -0.4, 0.9])
        n_B = np.array([1.4, -0.1, 1.8])

        angle_lower = 0.2 * math.pi
        angle_upper = 0.2 * math.pi

        self.ik_two_bodies.AddAngleBetweenVectorsConstraint(
            self.link1_frame, n_A, self.link2_frame, n_B, angle_lower, angle_upper)
        result = self.ik_two_bodies.get_mutable_prog().Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)

        q_val = self.prog.GetSolution(self.q)
        body1_quat = q_val[0:4]
        body2_quat = q_val[7:11]
        body1_rotmat = quaternionToRotationMatrix(body1_quat)
        body2_rotmat = quaternionToRotationMatrix(body2_quat)

        n_A_W = body1_rotmat.dot(n_A)
        n_B_W = body2_rotmat.dot(n_B)

        angle = math.acos(n_A_W.transpose().dot(n_B_W) /
                          (np.linalg.norm(n_A_W) * np.linalg.norm(n_B_W)))

        self.assertLess(math.fabs(angle - angle_lower), 1E-6)
