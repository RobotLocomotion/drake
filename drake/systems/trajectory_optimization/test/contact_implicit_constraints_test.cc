#include "drake/systems/trajectory_optimization/contact_implicit_constraints.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace {
// Test the general nonlinear complementary constraint
// g(x) = [x(0)² + x(0) * x(1); x(0) + 3 - x(2)²]
// h(x) = [x(1) - x(2) * x(0) + 1; x(2) - x(1)²]
// 0 ≤ g(x) ⊥ h(x) ≥ 0
// One solution is (1, -1, 2)
template <typename DerivedX, typename DerivedY>
typename std::enable_if<std::is_same<typename DerivedX::Scalar,
                                     typename DerivedY::Scalar>::value>::type
g_test(const Eigen::Ref<const DerivedX>& x, Eigen::Ref<DerivedY> y) {
  y << x(0) * x(0) + x(0) * x(1), x(0) + 3 - x(2) * x(2);
}

template <typename DerivedX, typename DerivedY>
typename std::enable_if<std::is_same<typename DerivedX::Scalar,
                                     typename DerivedY::Scalar>::value>::type
h_test(const Eigen::Ref<const DerivedX>& x, Eigen::Ref<DerivedY> y) {
  y << x(1) - x(2) * x(0) + 1, x(2) - x(1) * x(1);
}

GTEST_TEST(ContactImplicitConstraintsTest,
           NonlinearComplementaryConstraintTest) {
  GeneralNonlinearComplementaryConstraint::nonlinear_fun_double g_double{
      g_test<Eigen::VectorXd, Eigen::VectorXd>};
  GeneralNonlinearComplementaryConstraint::nonlinear_fun_double h_double{
      h_test<Eigen::VectorXd, Eigen::VectorXd>};
  GeneralNonlinearComplementaryConstraint::nonlinear_fun_autodiff g_autodiff{
      g_test<AutoDiffVecXd, AutoDiffVecXd>};
  GeneralNonlinearComplementaryConstraint::nonlinear_fun_autodiff h_autodiff{
      h_test<AutoDiffVecXd, AutoDiffVecXd>};
  GeneralNonlinearComplementaryConstraint dut{
      g_double, g_autodiff, h_double, h_autodiff, 2, 3, 0};
  solvers::MathematicalProgram prog;
  auto z = prog.NewContinuousVariables<3>();
  const auto new_constraints = dut.AddConstraintToProgram(&prog, z);
  // Check the size of the slack variables.
  EXPECT_EQ(std::get<3>(new_constraints).rows(), 4);
  // Check the linear constraint is just empty.
  EXPECT_EQ(std::get<1>(new_constraints).constraint()->A().rows(), 0);

  solvers::SolutionResult result = prog.Solve();
  EXPECT_EQ(result, solvers::SolutionResult::kSolutionFound);
  const auto z_val = prog.GetSolution(z);
  Eigen::Vector2d g_val;
  Eigen::Vector2d h_val;
  g_test<Eigen::Vector3d, Eigen::Vector2d>(z_val, g_val);
  h_test<Eigen::Vector3d, Eigen::Vector2d>(z_val, h_val);
  EXPECT_TRUE((g_val.array() >= 0).all());
  EXPECT_TRUE((h_val.array() >= 0).all());
  EXPECT_NEAR(g_val.dot(h_val), 0, 1E-6);
}
}  // namespace
}  // namespace systems
}  // namespace drake
