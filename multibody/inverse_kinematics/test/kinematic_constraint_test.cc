#include "drake/multibody/inverse_kinematics/kinematic_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"

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

template <typename T>
unique_ptr<MultibodyTree<T>>
// Test kinematic constraints on two free floating bodies.
class TwoBodiesConstraintTest : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO-ASSIGN(TwoBodiesConstraintTest)

 public:
   std::unique_ptr<MultibodyTree<AutoDiffXd>> two_bodies_;
}

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

TEST_F(IiwaKinematicConstraintTest, OrientationConstraint) {
  const double angle_bound{0.1 * M_PI};
  OrientationConstraint constraint(
      *iiwa_autodiff_, iiwa_link_frame_indices_[6], iiwa_link_frame_indices_[2],
      angle_bound,
      dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(context_autodiff_.get()));

  EXPECT_EQ(constraint.num_constraints(), 1);
  EXPECT_EQ(constraint.num_vars(), iiwa_autodiff_->num_positions());
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(),
                              Vector1d(2 * std::cos(angle_bound) - 1)));
  EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), Vector1d(1)));

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
