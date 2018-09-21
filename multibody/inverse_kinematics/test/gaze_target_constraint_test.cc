#include "drake/multibody/inverse_kinematics/gaze_target_constraint.h"

#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
TEST_F(IiwaKinematicConstraintTest, GazeTargetConstraint) {
  const FrameIndex frameA_idx{GetFrameIndex("iiwa_link_7")};
  const Eigen::Vector3d p_AS(0.1, 0.2, 0.3);
  const Eigen::Vector3d n_A(-0.1, 0.3, 0.4);
  const FrameIndex frameB_idx{GetFrameIndex("iiwa_link_3")};
  const Eigen::Vector3d p_BT(0.4, 0.2, -0.3);
  const double cone_half_angle{0.1 * M_PI};
  GazeTargetConstraint constraint(
      iiwa_autodiff_.tree(), frameA_idx, p_AS, n_A, frameB_idx, p_BT,
      cone_half_angle,
      dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(context_autodiff_.get()));

  EXPECT_EQ(constraint.num_constraints(), 2);
  EXPECT_EQ(constraint.num_vars(), iiwa_autodiff_.tree().num_positions());
  EXPECT_TRUE(
      CompareMatrices(constraint.lower_bound(), Eigen::Vector2d::Zero()));
  EXPECT_TRUE(CompareMatrices(
      constraint.upper_bound(),
      Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity())));

  Eigen::VectorXd q(iiwa_autodiff_.tree().num_positions());
  // arbitrary joint configuration.
  q << 0.1, 0.2, -0.3, 0.5, -0.2, -0.05, 0.34;
  const AutoDiffVecXd q_autodiff = math::initializeAutoDiff(q);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(q_autodiff, &y_autodiff);

  Vector3<AutoDiffXd> p_AT;
  iiwa_autodiff_.tree().CalcPointsPositions(
      *context_autodiff_, iiwa_autodiff_.tree().get_frame(frameB_idx),
      p_BT.cast<AutoDiffXd>(), iiwa_autodiff_.tree().get_frame(frameA_idx),
      &p_AT);
  const Vector3<AutoDiffXd> p_ST_A = p_AT - p_AS;
  Vector2<AutoDiffXd> y_autodiff_expected;
  const Eigen::Vector3d n_A_normalized = n_A.normalized();
  y_autodiff_expected(0) = p_ST_A.dot(n_A_normalized);
  y_autodiff_expected(1) =
      pow(p_ST_A.dot(n_A_normalized), 2) -
      std::pow(std::cos(cone_half_angle), 2) * p_ST_A.squaredNorm();
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, 1E-12);
}

TEST_F(TwoFreeBodiesConstraintTest, GazeTargetConstraint) {
  // All the numbers are chosen arbitrarily.
  const Eigen::Vector3d p_AS(0.2, -0.1, 0.3);
  const Eigen::Vector3d n_A(0.4, -0.3, 0.9);
  const Eigen::Vector3d p_BT(-0.02, 0.5, 1.3);
  const Eigen::Quaterniond body1_quaternion(Eigen::AngleAxisd(
      0.1 * M_PI, Eigen::Vector3d(0.2, -.9, 1.3).normalized()));
  const Eigen::Vector3d body1_position(0.5, 1.6, 2.3);
  const Eigen::Quaterniond body2_quaternion(Eigen::AngleAxisd(
      0.3 * M_PI, Eigen::Vector3d(0.1, -1.5, 2.4).normalized()));
  const Eigen::Vector3d body2_position(0.1, -2.3, 10.2);
  Eigen::Matrix<double, 14, 1> q;
  q << QuaternionToVectorWxyz(body1_quaternion), body1_position,
      QuaternionToVectorWxyz(body2_quaternion), body2_position;
  // Compute the angle between the vector p_ST and n_A;
  const Eigen::Vector3d p_AT =
      body1_quaternion.inverse() *
      (body2_quaternion * p_BT + body2_position - body1_position);
  const Eigen::Vector3d p_ST_A = p_AT - p_AS;
  const double angle =
      std::acos(p_ST_A.dot(n_A) / (p_ST_A.norm() * n_A.norm()));

  {
    GazeTargetConstraint good_constraint(
        two_bodies_autodiff_.tree(), body1_index_, p_AS, n_A, body2_index_,
        p_BT, angle + 0.01 * M_PI,
        dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(
            context_autodiff_.get()));
    EXPECT_TRUE(good_constraint.CheckSatisfied(q));
  }
  {
    GazeTargetConstraint bad_constraint(
        two_bodies_autodiff_.tree(), body1_index_, p_AS, n_A, body2_index_,
        p_BT, angle - 0.01 * M_PI,
        dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(
            context_autodiff_.get()));
    EXPECT_FALSE(bad_constraint.CheckSatisfied(q));
  }
}

TEST_F(IiwaKinematicConstraintTest, GazeTargetConstraintConstructorError) {
  // Check if the constructor for GazeTargetConstraint will throw errors if the
  // inputs are incorrect.
  const Eigen::Vector3d p_AS(1, 2, 3);
  const Eigen::Vector3d p_BT(2, 3, 4);
  const FrameIndex frameA_index = GetFrameIndex("iiwa_link_3");
  const FrameIndex frameB_index = GetFrameIndex("iiwa_link_4");
  // zero n_A
  EXPECT_THROW(
      GazeTargetConstraint(iiwa_autodiff_.tree(), frameA_index, p_AS,
                           Eigen::Vector3d::Zero(), frameB_index, p_BT, 0.1,
                           dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(
                               context_autodiff_.get())),
      std::invalid_argument);

  // wrong cone_half_angle
  EXPECT_THROW(
      GazeTargetConstraint(iiwa_autodiff_.tree(), frameA_index, p_AS,
                           Eigen::Vector3d::Ones(), frameB_index, p_BT, -0.1,
                           dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(
                               context_autodiff_.get())),
      std::invalid_argument);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
