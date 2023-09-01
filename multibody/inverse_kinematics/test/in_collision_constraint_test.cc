#include "drake/multibody/inverse_kinematics/in_collision_constraint.h"

#include <limits>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/compute_numerical_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/solvers/minimum_value_constraint.h"
#include "drake/solvers/test_utilities/check_constraint_eval_nonsymbolic.h"

namespace drake {
namespace multibody {
namespace {
TEST_F(TwoFreeSpheresTest, InCollisionConstraint) {
  // Test InCollision constraint with MBP.
  double minimum_distance_upper = 0.5;
  double normalizer = 1;
  auto diagram_context_double = diagram_double_->CreateDefaultContext();
  auto plant_context_double = &(
      plant_double_->GetMyMutableContextFromRoot(diagram_context_double.get()));
  InCollisionConstraint dut_double(plant_double_, minimum_distance_upper,
                                   normalizer, plant_context_double);
  Eigen::VectorXd q_double_good = Eigen::VectorXd::Zero(14);
  q_double_good.head<4>() << 1, 0, 0, 0;
  q_double_good.segment<4>(7) << 1, 0, 0, 0;
  q_double_good.tail<3>() << radius1_ + radius2_ + minimum_distance_upper - 0.1,
      0, 0;

  EXPECT_TRUE(dut_double.CheckSatisfied(q_double_good));

  Eigen::VectorXd q_double_bad = q_double_good;
  q_double_bad.tail<3>() << radius1_ + radius2_ + minimum_distance_upper + 0.1,
      0, 0;
  EXPECT_FALSE(dut_double.CheckSatisfied(q_double_bad));

  // Now construct InCollisionConstraint with AutoDiffXd. Make sure the
  // evaluation is the same as the constraint constructed with double.
  auto diagram_context_autodiff = diagram_autodiff_->CreateDefaultContext();
  auto plant_context_ad = &(plant_autodiff_->GetMyMutableContextFromRoot(
      diagram_context_autodiff.get()));
  InCollisionConstraint dut_ad(plant_autodiff_, minimum_distance_upper,
                               normalizer, plant_context_ad);

  // Use arbitrary gradient.
  Eigen::MatrixXd q_gradient =
      Eigen::MatrixXd(plant_double_->num_positions(), 2);
  q_gradient.col(0) = Eigen::VectorXd::LinSpaced(q_gradient.rows(), 0, 2);
  q_gradient.col(1) = Eigen::VectorXd::LinSpaced(q_gradient.rows(), 3, -1);
  const auto q_ad_good = math::InitializeAutoDiff(q_double_good, q_gradient);
  const auto q_ad_bad = math::InitializeAutoDiff(q_double_bad, q_gradient);

  Eigen::VectorXd y_double;
  Eigen::VectorXd y_double_expected;
  dut_double.Eval(q_double_good, &y_double);
  dut_ad.Eval(q_double_good, &y_double_expected);
  EXPECT_TRUE(CompareMatrices(y_double, y_double_expected));
  AutoDiffVecXd y_ad;
  AutoDiffVecXd y_ad_expected;
  dut_double.Eval(q_ad_good, &y_ad);
  dut_ad.Eval(q_ad_good, &y_ad_expected);
  const double tol = 1E-12;
  EXPECT_TRUE(CompareMatrices(math::ExtractValue(y_ad), y_double));
  EXPECT_TRUE(CompareMatrices(math::ExtractValue(y_ad),
                              math::ExtractValue(y_ad_expected), tol));
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y_ad),
                              math::ExtractGradient(y_ad_expected), tol));
}

TEST_F(SpheresAndWallsTest, InCollisionConstraintCollisionChecker) {
  // Test InCollisionConstraint with CollisionChecker.
  planning::CollisionCheckerParams params;
  params.model = builder_.Build();
  params.robot_model_instances.push_back(multibody::default_model_instance());
  auto quaternion_difference = [](const Eigen::Ref<const Eigen::Vector4d>& q1,
                                  const Eigen::Ref<const Eigen::Vector4d>& q2) {
    // Compute 1-cos(theta/2) where theta is the angle between the two
    // quaternions.
    return 1 - (Eigen::Quaterniond(q1(0), q1(1), q1(2), q1(3)).conjugate() *
                Eigen::Quaterniond(q2(0), q2(1), q2(2), q2(3)))
                   .w();
  };
  params.configuration_distance_function = [quaternion_difference](
                                               const Eigen::VectorXd& q1,
                                               const Eigen::VectorXd& q2) {
    return quaternion_difference(q1.head<4>(), q2.head<4>()) +
           quaternion_difference(q1.segment<4>(7), q2.segment<4>(7)) +
           (q1.segment<3>(4) - q2.segment<3>(4)).norm() +
           (q1.segment<3>(11) - q2.segment<3>(11)).norm();
  };
  params.edge_step_size = 0.05;
  planning::SceneGraphCollisionChecker collision_checker(std::move(params));
  auto collision_checker_context =
      collision_checker.MakeStandaloneModelContext();

  double minimum_distance_upper = 0.1;
  double normalizer = 2;
  InCollisionConstraint dut(&collision_checker, minimum_distance_upper,
                            normalizer, collision_checker_context.get());

  Eigen::VectorXd q =
      Eigen::VectorXd::Zero(collision_checker.plant().num_positions());
  q.head<4>() << 1, 0, 0, 0;
  q.segment<4>(7) << 1, 0, 0, 0;
  q.tail<3>() << wall_length_ / 2 + minimum_distance_upper + 0.5, 0, 0;
  EXPECT_FALSE(dut.CheckSatisfied(q));
  q.tail<3>() << radius_ * 2 + minimum_distance_upper - 0.1, 0, 0;
  EXPECT_TRUE(dut.CheckSatisfied(q));
}
}  // namespace
}  // namespace multibody
}  // namespace drake
