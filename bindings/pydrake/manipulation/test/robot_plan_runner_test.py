import unittest
import numpy as np

from pydrake.manipulation.robot_plan_runner import (
    PlanData,
    PlanSender,
    PlanType,
    RobotPlanRunner
)
from pydrake.trajectories import PiecewisePolynomial

class TestRobotPlanRunner(unittest.TestCase):
    def test_robot_plan_runner(self):
        # Check a plan_runner can be initialized
        plan_runner = RobotPlanRunner(control_period_sec = 1e-3)

    def test_plan_data(self):
        # Check the PlanData structure can be initialized with optional
        # trajectory arguments
        no_args = PlanData()
        empty_plan = PlanData(PlanType.kEmptyPlan)
        joint_space_plan_blank = PlanData(PlanType.kJointSpacePlan)
        joint_space_plan_pp = PlanData(
            PlanType.kJointSpacePlan,
            PiecewisePolynomial(np.array([[1.], [4.]])))

    def test_plan_sender(self):
        # Test PlanSender with no plans
        plan_sender = PlanSender([])
        self.assertEqual(plan_sender.get_all_plans_duration(), 7.0)

        # Test PlanSender with a PiecewisePolynomial and empty joint_traj field
        t_knots = np.array([0, 1, 2])
        q_knots = np.array([
            [0, 0, 0, 0, 0, 0, 0],
            [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
            [1, 1, 1, 1, 1, 1, 1]
        ]).T
        traj = PiecewisePolynomial.Cubic(
            t_knots, q_knots, np.zeros(7), np.zeros((7)))
        plan_1 = PlanData(PlanType.kJointSpacePlan, traj)
        plan_2 = PlanData(PlanType.kJointSpacePlan)

        plan_sender = PlanSender([plan_1, plan_2])
        self.assertEqual(plan_sender.get_all_plans_duration(), 10.0)

    def test_plan_type(self):
        # Check that all of the elements of the enum were spelled correctly.
        PlanType.kJointSpacePlan
        PlanType.kTaskSpacePlan
        PlanType.kEmptyPlan
