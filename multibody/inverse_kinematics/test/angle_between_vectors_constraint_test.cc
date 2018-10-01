#include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"

#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
TEST_F(IiwaKinematicConstraintTest, AngleBetweenVectorsConstraint) {
  const Eigen::Vector3d n_A(0.2, -0.3, 0.9);
  const Eigen::Vector3d n_B(1.6, -3.2, 1.2);
  const double angle_lower{0.1};
  const double angle_upper{0.5 * M_PI};
  const FrameIndex frameA_index = GetFrameIndex("iiwa_link_3");
  const FrameIndex frameB_index = GetFrameIndex("iiwa_link_7");
  AngleBetweenVectorsConstraint constraint(
      *iiwa_autodiff_, frameA_index, n_A, frameB_index, n_B, angle_lower,
      angle_upper,
      dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(context_autodiff_.get()));

  EXPECT_EQ(constraint.num_constraints(), 1);
  EXPECT_EQ(constraint.num_vars(), iiwa_autodiff_->num_positions());
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(),
                              Vector1d(std::cos(angle_upper))));
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(),
                              Vector1d(std::cos(angle_upper))));

  Eigen::VectorXd q(iiwa_autodiff_->num_positions());
  q << 0.2, -0.5, 0.1, 0.25, -0.4, 0.35, 0.24;
  const AutoDiffVecXd q_autodiff = math::initializeAutoDiff(q);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(q_autodiff, &y_autodiff);

  Vector1<AutoDiffXd> y_autodiff_expected;
  y_autodiff_expected(0) = n_A.normalized().dot(
      iiwa_autodiff_
          ->CalcRelativeTransform(*context_autodiff_,
                                  iiwa_autodiff_->get_frame(frameA_index),
                                  iiwa_autodiff_->get_frame(frameB_index))
          .linear() *
      n_B.normalized());
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, 1E-12);
}

TEST_F(TwoFreeBodiesConstraintTest, AngleBetweenVectorsConstraint) {
  const Eigen::Vector3d n_A(0.2, -0.3, 0.9);
  const Eigen::Vector3d n_B(1.6, -3.2, 1.2);
  const Eigen::Quaterniond body1_quaternion(Eigen::AngleAxisd(
      0.3 * M_PI, Eigen::Vector3d(0.1, 0.3, 0.2).normalized()));
  const Eigen::Quaterniond body2_quaternion(Eigen::AngleAxisd(
      -0.2 * M_PI, Eigen::Vector3d(0.4, 1.5, -0.2).normalized()));
  const Eigen::Vector3d body1_position(0.4, -0.02, 3.5);
  const Eigen::Vector3d body2_position(-.1, -2.3, 0.05);
  const Eigen::Vector3d n_A_W = body1_quaternion * n_A;
  const Eigen::Vector3d n_B_W = body2_quaternion * n_B;
  const double angle =
      std::acos(n_A_W.dot(n_B_W) / (n_A_W.norm() * n_B_W.norm()));

  Eigen::Matrix<double, 14, 1> q;
  q << QuaternionToVectorWxyz(body1_quaternion), body1_position,
      QuaternionToVectorWxyz(body2_quaternion), body2_position;
  {
    AngleBetweenVectorsConstraint good_constraint(
        *two_bodies_autodiff_, body1_index_, n_A, body2_index_, n_B,
        angle - 0.01, angle + 0.01,
        dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(
            context_autodiff_.get()));
    EXPECT_TRUE(good_constraint.CheckSatisfied(q));
  }

  {
    AngleBetweenVectorsConstraint bad_constraint(
        *two_bodies_autodiff_, body1_index_, n_A, body2_index_, n_B,
        angle - 0.02, angle - 0.01,
        dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(
            context_autodiff_.get()));
    EXPECT_FALSE(bad_constraint.CheckSatisfied(q));
  }
}

TEST_F(IiwaKinematicConstraintTest,
       AngleBetweenVectorsConstraintConstructorError) {
  const FrameIndex frameA_index = GetFrameIndex("iiwa_link_3");
  const FrameIndex frameB_index = GetFrameIndex("iiwa_link_7");
  // n_A being zero vector.
  EXPECT_THROW(AngleBetweenVectorsConstraint(
                   *iiwa_autodiff_, frameA_index, Eigen::Vector3d::Zero(),
                   frameB_index, Eigen::Vector3d::Ones(), 0.1, 0.2,
                   dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(
                       context_autodiff_.get())),
               std::invalid_argument);
  // n_B being zero vector.
  EXPECT_THROW(AngleBetweenVectorsConstraint(
                   *iiwa_autodiff_, frameA_index, Eigen::Vector3d::Ones(),
                   frameB_index, Eigen::Vector3d::Zero(), 0.1, 0.2,
                   dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(
                       context_autodiff_.get())),
               std::invalid_argument);
  // angle_lower < 0
  EXPECT_THROW(AngleBetweenVectorsConstraint(
                   *iiwa_autodiff_, frameA_index, Eigen::Vector3d::Ones(),
                   frameB_index, Eigen::Vector3d::Ones(), -0.1, 0.2,
                   dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(
                       context_autodiff_.get())),
               std::invalid_argument);
  // angle_upper < angle_lower
  EXPECT_THROW(AngleBetweenVectorsConstraint(
                   *iiwa_autodiff_, frameA_index, Eigen::Vector3d::Ones(),
                   frameB_index, Eigen::Vector3d::Zero(), 0.1, 0.09,
                   dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(
                       context_autodiff_.get())),
               std::invalid_argument);
  // angle_upper > pi
  EXPECT_THROW(AngleBetweenVectorsConstraint(
                   *iiwa_autodiff_, frameA_index, Eigen::Vector3d::Ones(),
                   frameB_index, Eigen::Vector3d::Zero(), 0.1, 1.1 * M_PI,
                   dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(
                       context_autodiff_.get())),
               std::invalid_argument);
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
