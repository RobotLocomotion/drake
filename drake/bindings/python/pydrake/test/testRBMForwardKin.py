import unittest
import numpy as np
import pydrake

class TestRBMForwardKin(unittest.TestCase):
    def testFK0(self):
        r = pydrake.rbm.RigidBodyManipulator("../../examples/Pendulum/Pendulum.urdf")

        kinsol = r.doKinematics(np.zeros((7,1)), np.zeros((7,1)))

        p, = r.forwardKin(kinsol, np.zeros((3,1)), 0, 1, 0, 0)
        print p

    def testFK1(self):
        r = pydrake.rbm.RigidBodyManipulator("../../examples/Pendulum/Pendulum.urdf")

        kinsol = r.doKinematics(np.zeros((7,1)), np.zeros((7,1)), 1)

        p, dp = r.forwardKin(kinsol, np.zeros((3,1)), 0, 1, 0, 1)
        print p
        print dp

if __name__ == '__main__':
    unittest.main()


