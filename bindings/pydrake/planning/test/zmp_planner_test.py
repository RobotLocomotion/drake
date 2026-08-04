import unittest

import numpy as np

from pydrake.planning import (
    ZmpPlanner,
)
from pydrake.trajectories import PiecewisePolynomial


class TestZmpPlanner(unittest.TestCase):
    def test_zmp_planner(self):
        height = 1
        x0 = [0, 0, 0, 0]
        time = [0, 1, 2, 3]
        zmp_knots = np.array([[0, 0], [0.2, -0.1], [0.4, 0.1], [0.4, 0.1]]).T
        zmp_d = PiecewisePolynomial.FirstOrderHold(time, zmp_knots)
        planner = ZmpPlanner()
        planner.Plan(
            zmp_d=zmp_d,
            x0=x0,
            height=height,
            gravity=9.8,
            Qy=np.eye(2),
            R=0.2 * np.eye(2),
        )
        self.assertTrue(planner.has_planned())
        self.assertEqual(planner.get_A().shape, (4, 4))
        self.assertEqual(planner.get_B().shape, (4, 2))
        self.assertEqual(planner.get_C().shape, (2, 4))
        self.assertEqual(planner.get_D().shape, (2, 2))
        np.testing.assert_almost_equal(planner.get_Qy(), np.eye(2))
        np.testing.assert_almost_equal(planner.get_R(), 0.2 * np.eye(2))
        sample_time = 0.1
        np.testing.assert_almost_equal(
            planner.get_desired_zmp(time=sample_time).reshape((2, 1)),
            zmp_d.value(sample_time),
        )
        np.testing.assert_almost_equal(
            planner.get_desired_zmp(time=sample_time).reshape((2, 1)),
            planner.get_desired_zmp().value(sample_time),
        )
        np.testing.assert_almost_equal(
            planner.get_nominal_com(time=sample_time).reshape((2, 1)),
            planner.get_nominal_com().value(sample_time),
        )
        np.testing.assert_almost_equal(
            planner.get_nominal_comd(time=sample_time).reshape((2, 1)),
            planner.get_nominal_comd().value(sample_time),
        )
        np.testing.assert_almost_equal(
            planner.get_nominal_comdd(time=sample_time).reshape((2, 1)),
            planner.get_nominal_comdd().value(sample_time),
        )
        self.assertEqual(planner.get_Vxx().shape, (4, 4))
        np.testing.assert_almost_equal(
            planner.get_Vx(time=sample_time).reshape((4, 1)),
            planner.get_Vx().value(sample_time),
        )
