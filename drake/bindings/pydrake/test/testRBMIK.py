import unittest
import numpy as np
import pydrake
from pydrake.solvers import ik


class TestRBMIK(unittest.TestCase):
    def testPostureConstraint(self):
        r = pydrake.rbm.RigidBodyManipulator("../examples/Pendulum/Pendulum.urdf")
        q = -0.9
        posture_constraint = ik.PostureConstraint(r)
        posture_constraint.setJointLimits(np.array([[6]], dtype=np.int32),
                                          np.array([[q]]),
                                          np.array([[q]]))
        # Choose a seed configuration (randomly) and a nominal configuration (at 0)
        q_seed = np.vstack((np.zeros((6,1)),
                            np.random.rand()))
        q_nom = np.vstack((np.zeros((6,1)),
                       0.))

        options = ik.IKoptions(r)
        results = pydrake.solvers.ik.inverseKinSimple(r,
                                      q_seed,
                                      q_nom,
                                      [posture_constraint],
                                      options)
        self.assertAlmostEqual(results.q_sol[6], q, 1e-9)

if __name__ == '__main__':
    unittest.main()
