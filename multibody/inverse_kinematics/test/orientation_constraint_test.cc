#include "drake/multibody/inverse_kinematics/orientation_constraint.h"

#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
TEST_F(IiwaKinematicConstraintTest, OrientationConstraint) {
  const double angle_bound{0.1 * M_PI};
  OrientationConstraint constraint(
      *iiwa_autodiff_, iiwa_link_frame_indices_[6], iiwa_link_frame_indices_[2],
      angle_bound,
      dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(context_autodiff_.get()));

  EXPECT_EQ(constraint.num_constraints(), 1);
  EXPECT_EQ(constraint.num_vars(), iiwa_autodiff_->num_positions());
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(),
                              Vector1d(2 * std::cos(angle_bound) + 1)));
  EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), Vector1d(3)));

  Eigen::VectorXd q(iiwa_autodiff_->num_positions());
  // Arbitrary joint angles.
  q << 0.1, 0.2, 0.3, 0.4, 0.5, -0.3, -0.2;
  const AutoDiffVecXd q_autodiff = math::initializeAutoDiff(q);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(q_autodiff, &y_autodiff);

  AutoDiffVecXd y_autodiff_expected(1);
  y_autodiff_expected(0) =
      iiwa_autodiff_
          ->CalcRelativeTransform(
              *context_autodiff_,
              iiwa_autodiff_->get_frame(iiwa_link_frame_indices_[6]),
              iiwa_autodiff_->get_frame(iiwa_link_frame_indices_[2]))
          .linear()
          .trace();
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, 1E-12);
}

TEST_F(TwoFreeBodiesConstraintTest, OrientationConstraint) {
  // Check if the orientation constraints are satisfied for two bodies with
  // given orientations
  OrientationConstraint constraint(
      *two_bodies_autodiff_, body1_index_, body2_index_, 0.1 * M_PI,
      dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(context_autodiff_.get()));

  // With q_satisfied, the angle between body1 and body2 is 0.09 * M_PI, it
  // should satisfy the orientation constraint.
  Eigen::Matrix<double, 14, 1> q_satisfied;
  // a is an arbitrary rotation axis.
  const Eigen::Vector3d a(1.0 / 3, 2.0 / 3, -2.0 / 3);
  const double theta_satisfied = 0.09 * M_PI;
  const Eigen::Quaterniond body1_quaternion(0.5, -0.5, 0.1, 0.7);
  Eigen::Quaterniond body2_quaternion =
      body1_quaternion * Eigen::AngleAxisd(theta_satisfied, a);
  q_satisfied << QuaternionToVector4(body1_quaternion), Eigen::Vector3d::Zero(),
      QuaternionToVector4(body2_quaternion), Eigen::Vector3d(0.2, 0.3, 0.4);

  EXPECT_TRUE(constraint.CheckSatisfied(q_satisfied));

  // With q_unsatisfied, the angle between body1 and body2 is 0.101 * M_PI, it
  // should not satisfy the orientation constraint.
  const double theta_unsatisfied = 0.101 * M_PI;
  body2_quaternion = body1_quaternion * Eigen::AngleAxisd(theta_unsatisfied, a);
  Eigen::Matrix<double, 14, 1> q_unsatisfied;
  q_unsatisfied << QuaternionToVector4(body1_quaternion),
      Eigen::Vector3d::Zero(), QuaternionToVector4(body2_quaternion),
      Eigen::Vector3d::Zero();
  EXPECT_FALSE(constraint.CheckSatisfied(q_unsatisfied));
}

TEST_F(IiwaKinematicConstraintTest, OrientationConstraintConstructionError) {
  // Throws a logic error for negative angle bound.
  EXPECT_THROW(
      OrientationConstraint(*iiwa_autodiff_, iiwa_link_frame_indices_[6],
                            iiwa_link_frame_indices_[2], -0.01,
                            dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(
                                context_autodiff_.get())),
      std::logic_error);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
