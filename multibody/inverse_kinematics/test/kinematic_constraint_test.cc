#include "drake/multibody/inverse_kinematics/kinematic_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

// We test kinematic constraints on two robots, an IIWA robot and two free
// bodies. The IIWA test confirms that the bounds and the Eval function of each
// constraint computes the expected result. The two free bodies test confirms
// that the equations in Eval function semantically makes the two free bodies to
// satisfy the kinematic constraints.
namespace drake {
namespace multibody {

template <typename DerivedA, typename DerivedB>
void CompareAutoDiffVectors(const Eigen::MatrixBase<DerivedA>& a,
                            const Eigen::MatrixBase<DerivedB>& b, double tol) {
  EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(a),
                              math::autoDiffToValueMatrix(b), tol));
  EXPECT_TRUE(CompareMatrices(math::autoDiffToGradientMatrix(a),
                              math::autoDiffToGradientMatrix(b), tol));
}

Eigen::Vector4d QuaternionToVector4(const Eigen::Quaterniond& q) {
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

class IiwaKinematicConstraintTest : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaKinematicConstraintTest)

  IiwaKinematicConstraintTest()
      : iiwa_autodiff_{benchmarks::kuka_iiwa_robot::MakeKukaIiwaModel<
            AutoDiffXd>(true /* finalized model. */)},
        iiwa_double_{benchmarks::kuka_iiwa_robot::MakeKukaIiwaModel<double>(
            true /* finalized model. */)},
        context_autodiff_(iiwa_autodiff_->CreateDefaultContext()),
        context_double_(iiwa_double_->CreateDefaultContext()) {
    for (int i = 0; i < 7; ++i) {
      iiwa_link_frame_indices_[i] =
          iiwa_autodiff_->GetBodyByName("iiwa_link_" + std::to_string(i + 1))
              .body_frame()
              .index();
    }
  }

 protected:
  std::unique_ptr<MultibodyTree<AutoDiffXd>> iiwa_autodiff_;
  std::unique_ptr<MultibodyTree<double>> iiwa_double_;
  std::array<FrameIndex, 7> iiwa_link_frame_indices_;
  std::unique_ptr<systems::LeafContext<AutoDiffXd>> context_autodiff_;
  std::unique_ptr<systems::LeafContext<double>> context_double_;
};

// Test kinematic constraints on two free floating bodies.
class TwoFreeBodiesConstraintTest : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TwoFreeBodiesConstraintTest)

  TwoFreeBodiesConstraintTest()
      : two_bodies_autodiff_(test::ConstructTwoFreeBodies<AutoDiffXd>()),
        two_bodies_double_(test::ConstructTwoFreeBodies<double>()),
        body1_index_(
            two_bodies_autodiff_->GetBodyByName("body1").body_frame().index()),
        body2_index_(
            two_bodies_autodiff_->GetBodyByName("body2").body_frame().index()),
        context_autodiff_(two_bodies_autodiff_->CreateDefaultContext()),
        context_double_(two_bodies_double_->CreateDefaultContext()) {}

  ~TwoFreeBodiesConstraintTest() override {}

 protected:
  std::unique_ptr<MultibodyTree<AutoDiffXd>> two_bodies_autodiff_;
  std::unique_ptr<MultibodyTree<double>> two_bodies_double_;
  FrameIndex body1_index_;
  FrameIndex body2_index_;
  std::unique_ptr<systems::LeafContext<AutoDiffXd>> context_autodiff_;
  std::unique_ptr<systems::LeafContext<double>> context_double_;
};

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

TEST_F(IiwaKinematicConstraintTest, GazeTargetConstraint) {
  const FrameIndex frameA_idx{iiwa_link_frame_indices_[6]};
  const Eigen::Vector3d p_AS(0.1, 0.2, 0.3);
  const Eigen::Vector3d n_A(-0.1, 0.3, 0.4);
  const FrameIndex frameB_idx{iiwa_link_frame_indices_[2]};
  const Eigen::Vector3d p_BT(0.4, 0.2, -0.3);
  const double cone_half_angle{0.1 * M_PI};
  GazeTargetConstraint constraint(
      *iiwa_autodiff_, frameA_idx, p_AS, n_A, frameB_idx, p_BT, cone_half_angle,
      dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(context_autodiff_.get()));

  EXPECT_EQ(constraint.num_constraints(), 2);
  EXPECT_EQ(constraint.num_vars(), iiwa_autodiff_->num_positions());
  EXPECT_TRUE(
      CompareMatrices(constraint.lower_bound(), Eigen::Vector2d::Zero()));
  EXPECT_TRUE(CompareMatrices(
      constraint.upper_bound(),
      Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity())));

  Eigen::VectorXd q(iiwa_autodiff_->num_positions());
  // arbitrary joint configuration.
  q << 0.1, 0.2, -0.3, 0.5, -0.2, -0.05, 0.34;
  const AutoDiffVecXd q_autodiff = math::initializeAutoDiff(q);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(q_autodiff, &y_autodiff);

  Vector3<AutoDiffXd> p_AT;
  iiwa_autodiff_->CalcPointsPositions(
      *context_autodiff_, iiwa_autodiff_->get_frame(frameB_idx),
      p_BT.cast<AutoDiffXd>(), iiwa_autodiff_->get_frame(frameA_idx), &p_AT);
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
  q << QuaternionToVector4(body1_quaternion), body1_position,
      QuaternionToVector4(body2_quaternion), body2_position;
  // Compute the angle between the vector p_ST and n_A;
  const Eigen::Vector3d p_AT =
      body1_quaternion.inverse() * (body2_quaternion * p_BT + body2_position -
      body1_position);
  const Eigen::Vector3d p_ST_A = p_AT - p_AS;
  const double angle =
      std::acos(p_ST_A.dot(n_A) / (p_ST_A.norm() * n_A.norm()));

  GazeTargetConstraint constraint_satisfied(
      *two_bodies_autodiff_, body1_index_, p_AS, n_A, body2_index_, p_BT,
      angle + 0.01 * M_PI,
      dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(context_autodiff_.get()));
  EXPECT_TRUE(constraint_satisfied.CheckSatisfied(q));

  GazeTargetConstraint constraint_unsatisfied(
      *two_bodies_autodiff_, body1_index_, p_AS, n_A, body2_index_, p_BT,
      angle - 0.01 * M_PI,
      dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(context_autodiff_.get()));
  EXPECT_FALSE(constraint_unsatisfied.CheckSatisfied(q));
}

TEST_F(IiwaKinematicConstraintTest, GazeTargetConstraintConstructorError) {
  // Check if the constructor for GazeTargetConstraint will throw errors if the
  // inputs are incorrect.
  const Eigen::Vector3d p_AS(1, 2, 3);
  const Eigen::Vector3d p_BT(2, 3, 4);
  // zero n_A
  EXPECT_THROW(GazeTargetConstraint(
                   *iiwa_autodiff_, iiwa_link_frame_indices_[2], p_AS,
                   Eigen::Vector3d::Zero(), iiwa_link_frame_indices_[3], p_BT,
                   0.1, dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(
                            context_autodiff_.get())),
               std::logic_error);

  // wrong cone_half_angle
  EXPECT_THROW(GazeTargetConstraint(
                   *iiwa_autodiff_, iiwa_link_frame_indices_[2], p_AS,
                   Eigen::Vector3d::Ones(), iiwa_link_frame_indices_[3], p_BT,
                   -0.1, dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(
                             context_autodiff_.get())),
               std::logic_error);
}
}  // namespace multibody
}  // namespace drake
