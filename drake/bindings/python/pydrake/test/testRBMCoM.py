from __future__ import print_function
import unittest
import numpy as np
import pydrake
import os.path

class TestRBMCoM(unittest.TestCase):
    def testCoM0(self):
        r = pydrake.rbm.RigidBodyTree(os.path.join(pydrake.getDrakePath(), "examples/Pendulum/Pendulum.urdf"))

        kinsol = r.doKinematics(np.zeros((7,1)), np.zeros((7,1)))

        c = r.centerOfMass(kinsol)

        print(c)


if __name__ == '__main__':
    unittest.main()