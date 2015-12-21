from __future__ import print_function
import unittest
import numpy as np
import pydrake
from pydrake.autodiff_utils import newAutoDiff
import os.path

class TestRBMForwardKin(unittest.TestCase):
    def testFK0(self):
        r = pydrake.rbm.RigidBodyTree(os.path.join(pydrake.getDrakePath(), "examples/Pendulum/Pendulum.urdf"))
        print("num positions:", r.num_positions)
        print("num velocities:", r.num_velocities)

        kinsol = r.doKinematics(np.zeros((7,1)), np.zeros((7,1)))

        p = r.forwardKin(kinsol, np.zeros((3,1)), 0, 1, 0)
        print(p)

    def testFK1(self):
        r = pydrake.rbm.RigidBodyTree(os.path.join(pydrake.getDrakePath(), "examples/Pendulum/Pendulum.urdf"))

        q = newAutoDiff(np.zeros((7,1)), np.eye(7, 7))
        v = newAutoDiff(np.zeros((7,1)), np.zeros((7, 7)))
        kinsol = r.doKinematics(q, v)

        point = np.ones((3,1))
        p = r.forwardKin(kinsol, point, 2, 0, 0)
        print(p)
        print(p.value())
        print(p.derivatives())

if __name__ == '__main__':
    unittest.main()


