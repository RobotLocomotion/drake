#include "drake/systems/trajectory_optimization/generalized_constraint_force_evaluator.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace {
// Construct a RigidBodyTree containing a four bar linkage.
std::unique_ptr<RigidBodyTree<double>> ConstructFourBarTree() {
  RigidBodyTree<double>* tree = new RigidBodyTree<double>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/simple_four_bar/FourBar.urdf"),
      multibody::joints::kFixed, tree);
  DRAKE_DEMAND(tree->get_num_actuators() != 0);
  return std::unique_ptr<RigidBodyTree<double>>(tree);
}

GTEST_TEST(GeneralizedConstraintForceEvaluatorTest, TestEval) {
  // Test the Eval function of GeneralizedConstraintForceEvaluator.
  auto tree = ConstructFourBarTree();

  auto cache_helper =
      std::make_shared<plants::KinematicsCacheWithVHelper<AutoDiffXd>>(*tree);

  PositionConstraintForceEvaluator evaluator(*tree, cache_helper);

  // Set q to some arbitrary number.
  const Eigen::VectorXd q =
      Eigen::VectorXd::LinSpaced(tree->get_num_positions(), 1, 5);
  const Eigen::VectorXd v =
      Eigen::VectorXd::LinSpaced(tree->get_num_velocities(), 0, 2);
  // Set lambda to some arbitrary number.
  const Eigen::VectorXd lambda =
      Eigen::VectorXd::LinSpaced(evaluator.num_lambda(), -3, 3);
  Eigen::VectorXd x(q.rows() + v.rows() + lambda.rows());
  x << q, v, lambda;
  const auto tx = math::initializeAutoDiff(x);
  AutoDiffVecXd ty;
  evaluator.Eval(tx, ty);
  EXPECT_EQ(ty.rows(), tree->get_num_velocities());
  KinematicsCache<double> kinsol = tree->CreateKinematicsCache();
  kinsol.initialize(q);
  tree->doKinematics(kinsol);
  const auto J = tree->positionConstraintsJacobian(kinsol);
  const Eigen::VectorXd y_expected = J.transpose() * lambda;
  EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(ty), y_expected,
                              1E-10, MatrixCompareType::absolute));
}
}  // namespace
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
