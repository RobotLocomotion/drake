#include "drake/systems/trajectory_optimization/joint_limit_constraint_force_evaluator.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/systems/trajectory_optimization/test/generalized_constraint_force_evaluator_test_util.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace {
GTEST_TEST(JointLimitConstraintForceEvaluatorTest, TestEval) {
  auto tree = ConstructFourBarTree();

  // Pretend that the four bar has a joint limit at joint 0.
  JointLimitConstraintForceEvaluator evaluator(*tree, 0);
  EXPECT_EQ(evaluator.lambda_size(), 2);

  // Doesn't need a non-lambda part to evaluate the Jacobian.
  const Eigen::VectorXd non_lambda(0);
  // Set lambda to some arbitrary number.
  const Eigen::VectorXd lambda =
      Eigen::VectorXd::LinSpaced(evaluator.lambda_size(), -3, 3);
  const Eigen::VectorXd x =
      evaluator.ComposeEvalInputVector(non_lambda, lambda);
  const auto tx = math::initializeAutoDiff(x);
  AutoDiffVecXd ty;
  evaluator.Eval(tx, &ty);

  Eigen::VectorXd y_expected =
      Eigen::VectorXd::Zero(tree->get_num_velocities());
  y_expected(0) =
      lambda(
          JointLimitConstraintForceEvaluator::LowerLimitForceIndexInLambda()) -
      lambda(
          JointLimitConstraintForceEvaluator::UpperLimitForceIndexInLambda());
  EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(ty), y_expected,
                              1E-10, MatrixCompareType::absolute));
}
}  // namespace
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
