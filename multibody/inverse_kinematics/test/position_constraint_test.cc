#include "drake/multibody/inverse_kinematics/position_constraint.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

using drake::systems::Context;

namespace drake {
namespace multibody {
namespace {
const double kInf = std::numeric_limits<double>::infinity();

AutoDiffVecXd EvalPositionConstraintAutoDiff(
    const Context<AutoDiffXd>& context, const MultibodyPlant<AutoDiffXd>& plant,
    const Frame<AutoDiffXd>& frameAbar, const math::RigidTransformd& X_AAbar,
    const Frame<AutoDiffXd>& frameB, const Vector3<AutoDiffXd>& p_BQ) {
  Vector3<AutoDiffXd> p_AbarQ;
  plant.CalcPointsPositions(context, frameB, p_BQ, frameAbar, &p_AbarQ);
  return X_AAbar.cast<AutoDiffXd>() * p_AbarQ;
}

TEST_F(IiwaKinematicConstraintTest, PositionConstraintGivenPBQ) {
  // Test PositionConstraint with p_BQ specified.
  const Eigen::Vector3d p_BQ(0.1, 0.2, 0.3);
  const Eigen::Vector3d p_AQ_lower(-0.2, -0.3, -0.4);
  const Eigen::Vector3d p_AQ_upper(0.2, 0.3, 0.4);
  const auto frameAbar_index = plant_->GetFrameByName("iiwa_link_7").index();
  const auto frameB_index = plant_->GetFrameByName("iiwa_link_3").index();
  const Frame<double>& frameAbar = plant_->get_frame(frameAbar_index);
  const math::RigidTransformd X_AAbar(
      math::RotationMatrixd(Eigen::AngleAxisd(
          0.2 * M_PI, Eigen::Vector3d(1.0 / 3, 2.0 / 3, -2.0 / 3))),
      Eigen::Vector3d(0.5, 0.2, -0.1));
  const Frame<double>& frameB = plant_->get_frame(frameB_index);

  auto check_constraint = [this, p_AQ_lower, p_AQ_upper, &frameB, p_BQ,
                           &frameAbar](
                              const PositionConstraint& dut,
                              const math::RigidTransformd& X_AAbar_val) {
    EXPECT_EQ(dut.num_vars(), this->plant_->num_positions());
    EXPECT_EQ(dut.num_constraints(), 3);
    EXPECT_EQ(dut.lower_bound(), p_AQ_lower);
    EXPECT_EQ(dut.upper_bound(), p_AQ_upper);

    // Now check if Eval function computes the right result.
    Eigen::VectorXd q(7);
    q << 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3;
    Eigen::VectorXd y;
    dut.Eval(q, &y);

    this->plant_->SetPositions(this->plant_context_, q);
    Eigen::Vector3d p_AbarQ;
    this->plant_->CalcPointsPositions(*(this->plant_context_), frameB, p_BQ,
                                      frameAbar, &p_AbarQ);
    const Eigen::Vector3d y_expected = X_AAbar_val * p_AbarQ;

    const double tol = 1E-12;
    EXPECT_TRUE(CompareMatrices(y, y_expected, tol));

    VectorX<AutoDiffXd> q_autodiff = math::InitializeAutoDiff(q);
    AutoDiffVecXd y_autodiff;
    dut.Eval(q_autodiff, &y_autodiff);
    this->plant_autodiff_->SetPositions(this->plant_context_autodiff_.get(),
                                        q_autodiff);
    Vector3<AutoDiffXd> y_autodiff_expected = EvalPositionConstraintAutoDiff(
        *(this->plant_context_autodiff_), *(this->plant_autodiff_),
        this->plant_autodiff_->GetFrameByName(frameAbar.name()), X_AAbar_val,
        this->plant_autodiff_->GetFrameByName(frameB.name()),
        p_BQ.cast<AutoDiffXd>());
    CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, tol);

    // Test with non-identity gradient for q_autodiff.
    q_autodiff =
        math::InitializeAutoDiff(q, MatrixX<double>::Ones(q.size(), 2));
    this->plant_autodiff_->SetPositions(this->plant_context_autodiff_.get(),
                                        q_autodiff);
    dut.Eval(q_autodiff, &y_autodiff);
    y_autodiff_expected = EvalPositionConstraintAutoDiff(
        *(this->plant_context_autodiff_), *(this->plant_autodiff_),
        this->plant_autodiff_->GetFrameByName(frameAbar.name()), X_AAbar_val,
        this->plant_autodiff_->GetFrameByName(frameB.name()),
        p_BQ.cast<AutoDiffXd>());
    CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, 1E-12);
  };
  // Constructs constraint with non-empty X_AbarA.
  PositionConstraint constraint(plant_, frameAbar, X_AAbar.inverse(),
                                p_AQ_lower, p_AQ_upper, frameB, p_BQ,
                                plant_context_);
  check_constraint(constraint, X_AAbar);

  // Constructs constraint with empty X_AAbar.
  PositionConstraint constraint_empty_X_AAbar(
      plant_, frameAbar, p_AQ_lower, p_AQ_upper, frameB, p_BQ, plant_context_);
  check_constraint(constraint_empty_X_AAbar, math::RigidTransformd::Identity());

  // Checks if the constraint constructed from MBP<ADS> gives the same result
  // as from MBP<double>.
  PositionConstraint constraint_from_autodiff(
      plant_autodiff_.get(), plant_autodiff_->get_frame(frameAbar_index),
      X_AAbar.inverse(), p_AQ_lower, p_AQ_upper,
      plant_autodiff_->get_frame(frameB_index), p_BQ,
      plant_context_autodiff_.get());
  Eigen::VectorXd q(7);
  q << 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3;
  // Set dq to arbitrary value.
  Eigen::Matrix<double, 7, 2> dq;
  for (int i = 0; i < 7; ++i) {
    dq(i, 0) = i * 2 + 1;
    dq(i, 1) = std::sin(i + 0.2);
  }
  /* tolerance for checking numerical gradient vs analytical gradient. The
   * numerical gradient is only accurate up to 4E-7 */
  // N.B. Using libsdformat9 with pose frame semantics requires an increase in
  // tolerance from 2E-7 to 4E-7.
  const double gradient_tol = 4E-7;
  TestKinematicConstraintEval(constraint, constraint_from_autodiff, q, dq,
                              gradient_tol);

  // Update bounds
  Eigen::Vector3d new_lb(-0.5, -kInf, 0);
  Eigen::Vector3d new_ub(kInf, kInf, 2);
  constraint.set_bounds(new_lb, new_ub);
  EXPECT_EQ(constraint.num_constraints(), 3);
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), new_lb));
  EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), new_ub));
  new_lb << -0.2, 1.5, 0.5;
  constraint.UpdateLowerBound(new_lb);
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), new_lb));
  EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), new_ub));
  new_ub << 2, kInf, 3;
  constraint.UpdateUpperBound(new_ub);
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), new_lb));
  EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), new_ub));
}

TEST_F(IiwaKinematicConstraintTest, PositionConstraintFreePBQ) {
  // Test PositionConstraint with p_BQ as decision variable.
  const Eigen::Vector3d p_AQ_lower(-0.2, -0.3, -0.4);
  const Eigen::Vector3d p_AQ_upper(0.2, 0.3, 0.4);
  const auto frameAbar_index = plant_->GetFrameByName("iiwa_link_7").index();
  const auto frameB_index = plant_->GetFrameByName("iiwa_link_3").index();
  const Frame<double>& frameAbar = plant_->get_frame(frameAbar_index);
  const math::RigidTransformd X_AAbar(
      math::RotationMatrixd(Eigen::AngleAxisd(
          0.2 * M_PI, Eigen::Vector3d(1.0 / 3, 2.0 / 3, -2.0 / 3))),
      Eigen::Vector3d(0.5, 0.2, -0.1));
  const Frame<double>& frameB = plant_->get_frame(frameB_index);

  auto check_constraint = [this, p_AQ_lower, p_AQ_upper, &frameB, &frameAbar](
                              const PositionConstraint& dut,
                              const math::RigidTransformd& X_AAbar_val) {
    EXPECT_EQ(dut.num_vars(), this->plant_->num_positions() + 3);
    EXPECT_EQ(dut.num_constraints(), 3);
    EXPECT_EQ(dut.lower_bound(), p_AQ_lower);
    EXPECT_EQ(dut.upper_bound(), p_AQ_upper);

    // Now check if Eval function computes the right result.
    Eigen::VectorXd q(7);
    q << 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3;
    const Eigen::Vector3d p_BQ(0.1, 0.2, 0.3);
    Eigen::VectorXd x(10);
    x << q, p_BQ;
    Eigen::VectorXd y;
    dut.Eval(x, &y);

    this->plant_->SetPositions(this->plant_context_, q);
    Eigen::Vector3d p_AbarQ;
    this->plant_->CalcPointsPositions(*(this->plant_context_), frameB, p_BQ,
                                      frameAbar, &p_AbarQ);
    const Eigen::Vector3d y_expected = X_AAbar_val * p_AbarQ;

    const double tol = 1E-12;
    EXPECT_TRUE(CompareMatrices(y, y_expected, tol));

    VectorX<AutoDiffXd> x_autodiff = math::InitializeAutoDiff(x);
    AutoDiffVecXd y_autodiff;
    dut.Eval(x_autodiff, &y_autodiff);
    EXPECT_TRUE(
        CompareMatrices(math::ExtractValue(y_autodiff), y_expected, tol));
    VectorX<AutoDiffXd> q_autodiff = x_autodiff.head(7);
    Vector3<AutoDiffXd> p_BQ_ad = x_autodiff.tail<3>();
    this->plant_autodiff_->SetPositions(this->plant_context_autodiff_.get(),
                                        q_autodiff);
    Vector3<AutoDiffXd> y_autodiff_expected = EvalPositionConstraintAutoDiff(
        *(this->plant_context_autodiff_), *(this->plant_autodiff_),
        this->plant_autodiff_->GetFrameByName(frameAbar.name()), X_AAbar_val,
        this->plant_autodiff_->GetFrameByName(frameB.name()), p_BQ_ad);
    CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, tol);

    // Test with non-identity gradient for q_autodiff.
    x_autodiff =
        math::InitializeAutoDiff(x, MatrixX<double>::Ones(x.size(), 2));
    q_autodiff = x_autodiff.head(7);
    p_BQ_ad = x_autodiff.tail<3>();
    this->plant_autodiff_->SetPositions(this->plant_context_autodiff_.get(),
                                        q_autodiff);
    dut.Eval(x_autodiff, &y_autodiff);
    y_autodiff_expected = EvalPositionConstraintAutoDiff(
        *(this->plant_context_autodiff_), *(this->plant_autodiff_),
        this->plant_autodiff_->GetFrameByName(frameAbar.name()), X_AAbar_val,
        this->plant_autodiff_->GetFrameByName(frameB.name()), p_BQ_ad);
    CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, 1E-12);
  };
  // Constructs constraint with non-empty X_AbarA.
  const math::RigidTransformd X_AbarA = X_AAbar.inverse();
  PositionConstraint constraint(plant_, frameAbar, X_AbarA, p_AQ_lower,
                                p_AQ_upper, frameB, std::nullopt,
                                plant_context_);
  check_constraint(constraint, X_AAbar);

  // Constructs constraint without X_AAbar.
  PositionConstraint constraint_without_X_AAbar(plant_, frameAbar, p_AQ_lower,
                                                p_AQ_upper, frameB,
                                                std::nullopt, plant_context_);
  check_constraint(constraint_without_X_AAbar,
                   math::RigidTransformd::Identity());
  // Checks if the constraint constructed from MBP<ADS> gives the same result
  // as from MBP<double>, with p_BQ as decision variable.
  PositionConstraint constraint_from_autodiff(
      plant_autodiff_.get(), plant_autodiff_->get_frame(frameAbar_index),
      X_AAbar.inverse(), p_AQ_lower, p_AQ_upper,
      plant_autodiff_->get_frame(frameB_index), std::nullopt,
      plant_context_autodiff_.get());
  Eigen::VectorXd x(10);
  x << 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3, 0.2, 0.3, 0.4;
  // Set dq to arbitrary value.
  Eigen::Matrix<double, 10, 2> dx;
  for (int i = 0; i < 10; ++i) {
    dx(i, 0) = i * 2 + 1;
    dx(i, 1) = std::sin(i + 0.2);
  }
  /* Tolerance for checking numerical gradient vs analytical gradient. The
   * numerical gradient is only accurate up to 4E-7. */
  // N.B. Using libsdformat9 with pose frame semantics requires an increase in
  // tolerance from 2E-7 to 4E-7.
  const double gradient_tol = 4E-7;
  TestKinematicConstraintEval(constraint, constraint_from_autodiff, x, dx,
                              gradient_tol);
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
  q << QuaternionToVectorWxyz(body1_quaternion), body1_position,
      QuaternionToVectorWxyz(body2_quaternion), body2_position;
  plant_->SetPositions(plant_context_, q);
  const Eigen::Vector3d p_BQ(0.2, 0.3, 0.4);
  Eigen::Vector3d p_AQ;
  plant_->CalcPointsPositions(*plant_context_, plant_->get_frame(body1_index_),
                              p_BQ, plant_->get_frame(body2_index_), &p_AQ);

  {
    PositionConstraint good_constraint(plant_, plant_->get_frame(body2_index_),
                                       p_AQ - Eigen::Vector3d::Constant(0.001),
                                       p_AQ + Eigen::Vector3d::Constant(0.001),
                                       plant_->get_frame(body1_index_), p_BQ,
                                       plant_context_);
    EXPECT_TRUE(good_constraint.CheckSatisfied(q));
  }
  {
    PositionConstraint bad_constraint(plant_, plant_->get_frame(body2_index_),
                                      p_AQ - Eigen::Vector3d::Constant(0.002),
                                      p_AQ - Eigen::Vector3d::Constant(0.001),
                                      plant_->get_frame(body1_index_), p_BQ,
                                      plant_context_);
    EXPECT_FALSE(bad_constraint.CheckSatisfied(q));
  }
}
}  // namespace
}  // namespace multibody
}  // namespace drake
