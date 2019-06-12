#include "drake/multibody/inverse_kinematics/distance_constraint.h"

#include "drake/math/compute_numerical_gradient.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"
#include "drake/solvers/test_utilities/check_constraint_eval_nonsymbolic.h"

namespace drake {
namespace multibody {
using solvers::test::CheckConstraintEvalNonsymbolic;

TEST_F(TwoFreeSpheresTest, Constructor) {
  const SortedPair<geometry::GeometryId> geometry_pair(sphere1_geometry_id_,
                                                       sphere2_geometry_id_);
  const DistanceConstraint dut(plant_double_, geometry_pair,
                               plant_context_double_, 0.1, 2);

  EXPECT_EQ(dut.num_vars(), plant_double_->num_positions());
  EXPECT_TRUE(CompareMatrices(dut.lower_bound(), Vector1d(0.1)));
  EXPECT_TRUE(CompareMatrices(dut.upper_bound(), Vector1d(2)));
}

TEST_F(TwoFreeSpheresTest, TestEval) {
  const SortedPair<geometry::GeometryId> geometry_pair(sphere1_geometry_id_,
                                                       sphere2_geometry_id_);
  const DistanceConstraint dut(plant_double_, geometry_pair,
                               plant_context_double_, 0.1, 2);

  Eigen::Matrix<double, 14, 1> q;
  // Set q to arbitrary value.
  q << 0.1, -0.2, 0.3, 0.4, -0.5, -0.2, 0.11, 0.12, 1.2, -0.1, -1.2, 0.3, -0.4,
      0.5;

  Eigen::VectorXd y;
  dut.Eval(q, &y);

  plant_double_->SetPositions(plant_context_double_, q);
  const math::RigidTransformd X_WB1 = plant_double_->CalcRelativeTransform(
      *plant_context_double_, plant_double_->world_frame(),
      plant_double_->get_frame(sphere1_index_));
  const math::RigidTransformd X_WB2 = plant_double_->CalcRelativeTransform(
      *plant_context_double_, plant_double_->world_frame(),
      plant_double_->get_frame(sphere2_index_));
  const math::RigidTransformd X_WS1 = X_WB1 * X_B1S1_;
  const math::RigidTransformd X_WS2 = X_WB2 * X_B2S2_;
  const double y_expected =
      (X_WS1.translation() - X_WS2.translation()).norm() - radius1_ - radius2_;
  EXPECT_TRUE(CompareMatrices(y, Vector1d(y_expected), 1E-12));

  Eigen::Matrix<AutoDiffXd, 14, 1> q_autodiff = math::initializeAutoDiff(q);
  CheckConstraintEvalNonsymbolic(dut, q_autodiff, 1E-12);
}
}  // namespace multibody
}  // namespace drake
