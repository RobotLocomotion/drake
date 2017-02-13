from __future__ import absolute_import, division, print_function

import unittest
import numpy as np
import pydrake
import pydrake.forwarddiff as fd
import os.path


class TestRBMForwardKin(unittest.TestCase):
    def test_value(self):
        r = pydrake.rbtree.RigidBodyTree(os.path.join(pydrake.getDrakePath(),
                                         "examples/Pendulum/Pendulum.urdf"))
        self.assertEqual(r.number_of_positions(), 7)
        self.assertEqual(r.number_of_velocities(), 7)

        kinsol = r.doKinematics(np.zeros((7, 1)), np.zeros((7, 1)))

        p = r.transformPoints(kinsol, np.zeros((3, 1)), 0, 1)
        self.assertTrue(np.allclose(p, np.zeros((3, 1))))

    def test_gradient(self):
        r = pydrake.rbtree.RigidBodyTree(os.path.join(pydrake.getDrakePath(),
                                         "examples/Pendulum/Pendulum.urdf"))

        def do_transform(q):
            kinsol = r.doKinematics(q)
            point = np.ones((3, 1))
            return r.transformPoints(kinsol, point, 2, 0)

        q = np.zeros(7)

        value = do_transform(q)
        self.assertTrue(np.allclose(value, np.ones((3, 1))))

        g = fd.jacobian(do_transform, q)
        self.assertTrue(np.allclose(g,
                                    np.array([[[1, 0, 0, 0, 1, -1, 1]],
                                              [[0, 1, 0, -1, 0, 1, 0]],
                                              [[0, 0, 1, 1, -1, 0, -1]]])))

    def test_relative_transform(self):
        r = pydrake.rbtree.RigidBodyTree(os.path.join(pydrake.getDrakePath(),
                                         "examples/Pendulum/Pendulum.urdf"))


        q = np.zeros(7)
        q[6] = np.pi / 2
        kinsol = r.doKinematics(q)
        T = r.relativeTransform(kinsol, 1, 2)
        self.assertTrue(np.allclose(T,
                                    np.array([[0, 0, 1, 0],
                                              [0, 1, 0, 0],
                                              [-1, 0, 0, 0],
                                              [0, 0, 0, 1]])))





if __name__ == '__main__':
    unittest.main()
