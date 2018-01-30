from __future__ import absolute_import, division, print_function

import unittest
import numpy as np
import pydrake
from pydrake.rbtree import RigidBodyTree, FloatingBaseType
import os.path


class TestRBTCoM(unittest.TestCase):
    def testCoM0(self):
        r = RigidBodyTree(os.path.join(pydrake.getDrakePath(),
                          "examples/pendulum/Pendulum.urdf"))

        kinsol = r.doKinematics(np.zeros((7, 1)), np.zeros((7, 1)))

        c = r.centerOfMass(kinsol)

        self.assertTrue(np.allclose(c.flat, [0.0, 0.0, -0.2425], atol=1e-4))

    def testCoMBody(self):
        r = RigidBodyTree(os.path.join(pydrake.getDrakePath(),
                          "examples/pendulum/Pendulum.urdf"))

        kinsol = r.doKinematics(np.zeros((7, 1)), np.zeros((7, 1)))

        arm_com = r.FindBody("arm_com")
        c = arm_com.get_center_of_mass()

        self.assertTrue(np.allclose(c.flat, [0.0, 0.0, -0.5], atol=1e-4))

    def testCoMJacobian(self):
        r = RigidBodyTree(os.path.join(pydrake.getDrakePath(),
                          "examples/pendulum/Pendulum.urdf"))
        q = r.getRandomConfiguration()
        kinsol = r.doKinematics(q, np.zeros((7, 1)))
        J = r.centerOfMassJacobian(kinsol)

        self.assertTrue(np.shape(J) == (3, 7))

        q = r.getZeroConfiguration()
        kinsol = r.doKinematics(q, np.zeros((7, 1)))
        J = r.centerOfMassJacobian(kinsol)

        self.assertTrue(
            np.allclose(J.flat, [1., 0., 0., 0., -0.2425, 0., -0.25,
                                 0., 1., 0., 0.2425, 0., 0., 0.,
                                 0., 0., 1., 0., 0., 0., 0.], atol=1e-4))

    def test_dynamics_api(self):
        urdf_path = os.path.join(
            pydrake.getDrakePath(), "examples/pendulum/Pendulum.urdf")
        r = RigidBodyTree(
            urdf_path, floating_base_type=FloatingBaseType.kRollPitchYaw)

        def assert_sane(x, nonzero=True):
            self.assertTrue(np.all(np.isfinite(x)))
            if nonzero:
                self.assertTrue(np.any(x != 0))

        num_q = num_v = 7
        q = np.zeros(num_q)
        v = np.zeros(num_v)
        # Update kinematics.
        kinsol = r.doKinematics(q, v)
        # Sanity checks:
        # - Actuator map.
        self.assertEquals(r.B.shape, (num_v, 1))
        B_expected = np.zeros((num_v, 1))
        B_expected[-1] = 1
        self.assertTrue(np.allclose(r.B, B_expected))
        # - Mass matrix.
        H = r.massMatrix(kinsol)
        self.assertEquals(H.shape, (num_v, num_v))
        assert_sane(H)
        self.assertTrue(np.allclose(H[-1, -1], 0.25))
        # - Bias terms.
        C = r.dynamicsBiasTerm(kinsol, {})
        self.assertEquals(C.shape, (num_v,))
        assert_sane(C)
        # - Inverse dynamics.
        vd = np.zeros(num_v)
        tau = r.inverseDynamics(kinsol, {}, vd)
        assert_sane(tau)
        # - Friction torques.
        friction_torques = r.frictionTorques(v)
        self.assertTrue(friction_torques.shape, (num_v,))
        assert_sane(friction_torques, nonzero=False)


if __name__ == '__main__':
    unittest.main()
