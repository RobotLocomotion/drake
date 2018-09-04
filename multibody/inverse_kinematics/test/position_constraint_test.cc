#include "drake/multibody/inverse_kinematics/position_constraint.h"

#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"
namespace drake {
namespace multibody {
namespace internal {
TEST_F(IiwaKinematicConstraintTest, PositionConstraint) {
  const Eigen::Vector3d p_BQ(0.1, 0.2, 0.3);
  const Eigen::Vector3d p_AQ_lower(-0.2, -0.3, -0.4);
  const Eigen::Vector3d p_AQ_upper(0.2, 0.3, 0.4);
  PositionConstraint constraint1(
      *iiwa_autodiff_, iiwa_link_frame_indices_[6], p_BQ,
      iiwa_link_frame_indices_[2], p_AQ_lower, p_AQ_upper,
      dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(context_autodiff_.get()));

  EXPECT_EQ(constraint1.num_vars(), iiwa_autodiff_->num_positions());
  EXPECT_EQ(constraint1.num_constraints(), 3);
  EXPECT_EQ(constraint1.lower_bound(), p_AQ_lower);
  EXPECT_EQ(constraint1.upper_bound(), p_AQ_upper);

  // Now check if Eval function computes the right result.
  Eigen::VectorXd q(7);
  q << 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3;
  Eigen::VectorXd y;
  constraint1.Eval(q, &y);

  Eigen::MatrixXd y_expected(3, 1);

  auto mbt_context_double =
      dynamic_cast<MultibodyTreeContext<double>*>(context_double_.get());
  mbt_context_double->get_mutable_positions() = q;
  iiwa_double_->CalcPointsPositions(
      *context_double_, iiwa_double_->get_frame(iiwa_link_frame_indices_[6]),
      p_BQ, iiwa_double_->get_frame(iiwa_link_frame_indices_[2]), &y_expected);
  const double tol = 1E-12;
  EXPECT_TRUE(CompareMatrices(y, y_expected, tol));

  const VectorX<AutoDiffXd> q_autodiff = math::initializeAutoDiff(q);
  AutoDiffVecXd y_autodiff;
  constraint1.Eval(q_autodiff, &y_autodiff);
  AutoDiffVecd<Eigen::Dynamic, 3> p_BQ_autodiff;
  Vector3<AutoDiffXd> y_autodiff_expected;
  iiwa_autodiff_->CalcPointsPositions(
      *context_autodiff_,
      iiwa_autodiff_->get_frame(iiwa_link_frame_indices_[6]),
      p_BQ.cast<AutoDiffXd>(),
      iiwa_autodiff_->get_frame(iiwa_link_frame_indices_[2]),
      &y_autodiff_expected);
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, tol);
}

TEST_F(TwoFreeBodiesConstraintTest, PositionConstraint) {
  // Given two free bodies with some given (arbitrary) poses, check if the poses
  // satisfy given position constraints.
  const Eigen::Quaterniond body1_quaternion(Eigen::AngleAxisd(
      0.3 * M_PI, Eigen::Vector3d(0.1, 0.3, 0.2).normalized()));
  const Eigen::Quaterniond body2_quaternion(Eigen::AngleAxisd(
      -0.2 * M_PI, Eigen::Vector3d(0.4, 1.5, -0.2).normalized()));
  const Eigen::Vector3d body1_position(0.4, -0.02, 3.5);
  const Eigen::Vector3d body2_position(-.1, -2.3, 0.05);
  Eigen::Matrix<double, 14, 1> q;
  q << QuaternionToVector4(body1_quaternion), body1_position,
      QuaternionToVector4(body2_quaternion), body2_position;
  dynamic_cast<MultibodyTreeContext<double>*>(context_double_.get())
      ->get_mutable_positions() = q;
  const Eigen::Vector3d p_BQ(0.2, 0.3, 0.4);
  Eigen::Vector3d p_AQ;
  two_bodies_double_->CalcPointsPositions(
      *context_double_, two_bodies_double_->get_frame(body1_index_), p_BQ,
      two_bodies_double_->get_frame(body2_index_), &p_AQ);

  PositionConstraint constraint_satisfied(
      *two_bodies_autodiff_, body1_index_, p_BQ, body2_index_,
      p_AQ - Eigen::Vector3d::Constant(0.001),
      p_AQ + Eigen::Vector3d::Constant(0.001),
      dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(context_autodiff_.get()));
  EXPECT_TRUE(constraint_satisfied.CheckSatisfied(q));
  PositionConstraint constraint_unsatisfied(
      *two_bodies_autodiff_, body1_index_, p_BQ, body2_index_,
      p_AQ - Eigen::Vector3d::Constant(0.002),
      p_AQ - Eigen::Vector3d::Constant(0.001),
      dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(context_autodiff_.get()));
  EXPECT_FALSE(constraint_unsatisfied.CheckSatisfied(q));
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
