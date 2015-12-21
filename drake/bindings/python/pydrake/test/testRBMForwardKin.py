from __future__ import print_function
import unittest
import numpy as np
import pydrake
import os.path

class TestRBMForwardKin(unittest.TestCase):
    def testFK0(self):
        r = pydrake.rbm.RigidBodyTree(os.path.join(pydrake.getDrakePath(), "examples/Pendulum/Pendulum.urdf"))

        kinsol = r.doKinematics(np.zeros((7,1)), np.zeros((7,1)))

        p = r.forwardKin(kinsol, np.zeros((3,1)), 0, 1, 0)
        print(p)

    def testFK1(self):
        r = pydrake.rbm.RigidBodyTree(os.path.join(pydrake.getDrakePath(), "examples/Pendulum/Pendulum.urdf"))

        kinsol = r.doKinematics(np.zeros((7,1)), np.zeros((7,1)))

        p = r.forwardKin(kinsol, np.zeros((3,1)), 0, 1, 0)
        print(p)

if __name__ == '__main__':
    unittest.main()


