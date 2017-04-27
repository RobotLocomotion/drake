from __future__ import absolute_import, division, print_function

import unittest
import numpy as np
import pydrake
from pydrake.solvers import ik
import os.path


class TestRBTIK(unittest.TestCase):
    def testPostureConstraint(self):
        r = pydrake.rbtree.RigidBodyTree(
            os.path.join(pydrake.getDrakePath(),
                         "examples/Pendulum/Pendulum.urdf"))
        q = -0.9
        posture_constraint = ik.PostureConstraint(r)
        posture_constraint.setJointLimits(np.array([[6]], dtype=np.int32),
                                          np.array([[q]]),
                                          np.array([[q]]))
        # Choose a seed configuration (randomly) and a nominal configuration
        # (at 0)
        q_seed = np.vstack((np.zeros((6, 1)),
                            0.8147))
        q_nom = np.vstack((np.zeros((6, 1)),
                           0.))

        options = ik.IKoptions(r)
        results = ik.InverseKin(
            r, q_seed, q_nom, [posture_constraint], options)
        self.assertEqual(results.info[0], 1)
        self.assertAlmostEqual(results.q_sol[0][6], q, 1e-9)

        # Run the tests again both pointwise and as a trajectory to
        # validate the interfaces.
        t = np.array([0., 1.])
        q_seed_array = np.transpose(np.array([q_seed, q_seed]))[0]
        q_nom_array = np.transpose(np.array([q_nom, q_nom]))[0]

        results = ik.InverseKinPointwise(r, t, q_seed_array, q_nom_array,
                                         [posture_constraint], options)
        self.assertEqual(len(results.info), 2)
        self.assertEqual(len(results.q_sol), 2)
        self.assertEqual(results.info[0], 1)

        # The pointwise result will go directly to the constrained
        # value.
        self.assertAlmostEqual(results.q_sol[0][6], q, 1e-9)
        self.assertEqual(results.info[1], 1)
        self.assertAlmostEqual(results.q_sol[1][6], q, 1e-9)

        results = ik.InverseKinTraj(r, t, q_seed_array, q_nom_array,
                                    [posture_constraint], options)
        self.assertEqual(len(results.info), 1)
        self.assertEqual(len(results.q_sol), 2)
        self.assertEqual(results.info[0], 1)

        # The trajectory result starts at the initial value and moves
        # to the constrained value.
        self.assertAlmostEqual(results.q_sol[0][6], q_seed[6], 1e-9)
        self.assertAlmostEqual(results.q_sol[1][6], q, 1e-9)


if __name__ == '__main__':
    unittest.main()
