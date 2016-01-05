import unittest
import numpy as np
import pydrake
from pydrake.solvers import ik
import os.path

class TestRBTIK(unittest.TestCase):
    def testPostureConstraint(self):
        r = pydrake.rbtree.RigidBodyTree(os.path.join(pydrake.getDrakePath(), "examples/Pendulum/Pendulum.urdf"))
        q = -0.9
        posture_constraint = ik.PostureConstraint(r)
        posture_constraint.setJointLimits(np.array([[6]], dtype=np.int32),
                                          np.array([[q]]),
                                          np.array([[q]]))
        # Choose a seed configuration (randomly) and a nominal configuration (at 0)
        q_seed = np.vstack((np.zeros((6,1)),
                            0.8147))
        q_nom = np.vstack((np.zeros((6,1)),
                       0.))

        options = ik.IKoptions(r)
        results = ik.inverseKinSimple(r,
                                      q_seed,
                                      q_nom,
                                      [posture_constraint],
                                      options)
        self.assertAlmostEqual(results.q_sol[6], q, 1e-9)

if __name__ == '__main__':
    unittest.main()
