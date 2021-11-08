#include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"

#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

using drake::math::RotationMatrixd;
using drake::systems::Context;

namespace drake {
namespace multibody {
namespace {
AutoDiffVecXd EvalAngleBetweenVectorsConstraint(
    const Context<AutoDiffXd>& context, const MultibodyPlant<AutoDiffXd>& plant,
    const Frame<AutoDiffXd>& frameA, const Vector3<double>& n_A,
    const Frame<AutoDiffXd>& frameB, const Vector3<double>& n_B) {
  Vector1<AutoDiffXd> y_autodiff;
  const math::RotationMatrix<AutoDiffXd> R_AB =
      plant.CalcRelativeRotationMatrix(context, frameA, frameB);
  y_autodiff(0) = n_A.normalized().dot(R_AB.matrix() * n_B.normalized());
  return y_autodiff;
}

TEST_F(IiwaKinematicConstraintTest, AngleBetweenVectorsConstraint) {
  const Eigen::Vector3d n_A(0.2, -0.3, 0.9);
  const Eigen::Vector3d n_B(1.6, -3.2, 1.2);
  const double angle_lower{0.1};
  const double angle_upper{0.5 * M_PI};
  const auto frameA_index = plant_->GetFrameByName("iiwa_link_3").index();
  const auto frameB_index = plant_->GetFrameByName("iiwa_link_7").index();
  const Frame<double>& frameA = plant_->get_frame(frameA_index);
  const Frame<double>& frameB = plant_->get_frame(frameB_index);
  AngleBetweenVectorsConstraint constraint(plant_, frameA, n_A, frameB, n_B,
                                           angle_lower, angle_upper,
                                           plant_context_);

  EXPECT_EQ(constraint.num_constraints(), 1);
  EXPECT_EQ(constraint.num_vars(), plant_autodiff_->num_positions());
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(),
                              Vector1d(std::cos(angle_upper))));
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(),
                              Vector1d(std::cos(angle_upper))));

  Eigen::VectorXd q(plant_autodiff_->num_positions());
  q << 0.2, -0.5, 0.1, 0.25, -0.4, 0.35, 0.24;
  AutoDiffVecXd q_autodiff = math::InitializeAutoDiff(q);
  plant_autodiff_->GetMutablePositions(plant_context_autodiff_.get()) =
      q_autodiff;
  AutoDiffVecXd y_autodiff;
  constraint.Eval(q_autodiff, &y_autodiff);

  Vector1<AutoDiffXd> y_autodiff_expected = EvalAngleBetweenVectorsConstraint(
      *plant_context_autodiff_, *plant_autodiff_,
      plant_autodiff_->GetFrameByName(frameA.name()), n_A,
      plant_autodiff_->GetFrameByName(frameB.name()), n_B);
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, 1E-12);

  // Test with non-identity gradient for q_autodiff.
  q_autodiff = math::InitializeAutoDiff(q, MatrixX<double>::Ones(q.size(), 2));
  plant_autodiff_->GetMutablePositions(plant_context_autodiff_.get()) =
      q_autodiff;
  constraint.Eval(q_autodiff, &y_autodiff);
  y_autodiff_expected = EvalAngleBetweenVectorsConstraint(
      *plant_context_autodiff_, *plant_autodiff_,
      plant_autodiff_->GetFrameByName(frameA.name()), n_A,
      plant_autodiff_->GetFrameByName(frameB.name()), n_B);
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, 1E-12);

  // Checks if the constraint constructed from MBP<ADS> gives the same result
  // as from MBP<double>.
  const AngleBetweenVectorsConstraint constraint_from_autodiff(
      plant_autodiff_.get(), plant_autodiff_->get_frame(frameA_index), n_A,
      plant_autodiff_->get_frame(frameB_index), n_B, angle_lower, angle_upper,
      plant_context_autodiff_.get());
  // Set dq to arbitrary value.
  Eigen::Matrix<double, 7, 2> dq;
  for (int i = 0; i < 7; ++i) {
    dq(i, 0) = i * 2 + 1;
    dq(i, 1) = std::sin(i + 0.2);
  }
  /* tolerance for checking numerical gradient vs analytical gradient. The
   * numerical gradient is only accurate up to 1E-6 */
  const double gradient_tol = 1E-6;
  TestKinematicConstraintEval(constraint, constraint_from_autodiff, q, dq,
                              gradient_tol);
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
        plant_, plant_->get_frame(body1_index_), n_A,
        plant_->get_frame(body2_index_), n_B, angle - 0.01, angle + 0.01,
        plant_context_);
    EXPECT_TRUE(good_constraint.CheckSatisfied(q));
  }

  {
    AngleBetweenVectorsConstraint bad_constraint(
        plant_, plant_->get_frame(body1_index_), n_A,
        plant_->get_frame(body2_index_), n_B, angle - 0.02, angle - 0.01,
        plant_context_);
    EXPECT_FALSE(bad_constraint.CheckSatisfied(q));
  }
}

TEST_F(IiwaKinematicConstraintTest,
       AngleBetweenVectorsConstraintConstructorError) {
  const Frame<double>& frameA = plant_->GetFrameByName("iiwa_link_3");
  const Frame<double>& frameB = plant_->GetFrameByName("iiwa_link_7");
  // n_A being zero vector.
  EXPECT_THROW(AngleBetweenVectorsConstraint(
                   plant_, frameA, Eigen::Vector3d::Zero(), frameB,
                   Eigen::Vector3d::Ones(), 0.1, 0.2, plant_context_),
               std::invalid_argument);
  // n_B being zero vector.
  EXPECT_THROW(AngleBetweenVectorsConstraint(
                   plant_, frameA, Eigen::Vector3d::Ones(), frameB,
                   Eigen::Vector3d::Zero(), 0.1, 0.2, plant_context_),
               std::invalid_argument);
  // angle_lower < 0
  EXPECT_THROW(AngleBetweenVectorsConstraint(
                   plant_, frameA, Eigen::Vector3d::Ones(), frameB,
                   Eigen::Vector3d::Ones(), -0.1, 0.2, plant_context_),
               std::invalid_argument);
  // angle_upper < angle_lower
  EXPECT_THROW(AngleBetweenVectorsConstraint(
                   plant_, frameA, Eigen::Vector3d::Ones(), frameB,
                   Eigen::Vector3d::Zero(), 0.1, 0.09, plant_context_),
               std::invalid_argument);
  // angle_upper > pi
  EXPECT_THROW(AngleBetweenVectorsConstraint(
                   plant_, frameA, Eigen::Vector3d::Ones(), frameB,
                   Eigen::Vector3d::Zero(), 0.1, 1.1 * M_PI, plant_context_),
               std::invalid_argument);
}
}  // namespace
}  // namespace multibody
}  // namespace drake
