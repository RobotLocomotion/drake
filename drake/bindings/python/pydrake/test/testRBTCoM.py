from __future__ import absolute_import, division, print_function

import unittest
import numpy as np
import pydrake
import os.path


class TestRBTCoM(unittest.TestCase):
    def testCoM0(self):
        r = pydrake.rbtree.RigidBodyTree(os.path.join(pydrake.getDrakePath(),
                                         "examples/Pendulum/Pendulum.urdf"))

        kinsol = r.doKinematics(np.zeros((7, 1)), np.zeros((7, 1)))

        c = r.centerOfMass(kinsol)

        self.assertTrue(np.allclose(c.flat, [0.0, 0.0, -0.2425], atol=1e-4))

    def testCoMJacobian(self):
        r = pydrake.rbtree.RigidBodyTree(os.path.join(pydrake.getDrakePath(),
                                         "examples/Pendulum/Pendulum.urdf"))
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


if __name__ == '__main__':
    unittest.main()
