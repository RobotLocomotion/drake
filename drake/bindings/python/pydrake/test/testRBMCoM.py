import unittest
import numpy as np
import pydrake

class TestRBMCoM(unittest.TestCase):
    def testCoM0(self):
        r = pydrake.rbm.RigidBodyManipulator("../../examples/Pendulum/Pendulum.urdf")

        kinsol = r.doKinematics(np.zeros((7,1)), np.zeros((7,1)))

        c, = r.centerOfMass(kinsol, 0)

        print c

    def testCoM1(self):
        r = pydrake.rbm.RigidBodyManipulator("../../examples/Pendulum/Pendulum.urdf")

        kinsol = r.doKinematics(np.zeros((7,1)), np.zeros((7,1)), 1)

        c, dc = r.centerOfMass(kinsol, 1)

        print c
        print dc

    def testCoM2(self):
        r = pydrake.rbm.RigidBodyManipulator("../../examples/Pendulum/Pendulum.urdf")

        kinsol = r.doKinematics(np.zeros((7,1)), np.zeros((7,1)), 2)

        try:
            c, dc, ddc = r.centerOfMass(kinsol, 2)
        except RuntimeError as e:
            pass
        else:
            raise RuntimeError("should have thrown a runtime error, since not enough gradients are available")



if __name__ == '__main__':
    unittest.main()