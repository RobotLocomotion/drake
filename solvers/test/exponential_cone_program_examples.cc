#include "drake/solvers/test/exponential_cone_program_examples.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

namespace drake {
namespace solvers {
namespace test {
const double kInf = std::numeric_limits<double>::infinity();

void ExponentialConeTrivialExample(const SolverInterface& solver, double tol) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>()(0);
  auto y = prog.NewContinuousVariables<1>()(0);
  auto z = prog.NewContinuousVariables<1>()(0);
  prog.AddExponentialConeConstraint(Vector3<symbolic::Expression>(x, y, z));
  prog.AddLinearEqualityConstraint(y + z == 1);
  prog.AddLinearCost(x);

  MathematicalProgramResult result;
  solver.Solve(prog, {}, {}, &result);
  EXPECT_TRUE(result.is_success());
  // Check the solution. We can rewrite the problem as a single variable
  // optimization min y * exp(1 - y / y). The gradient of this cost function is
  // (y - 1)/y * exp(1-y / y). The optimal value is when the gradient is 0.
  // Namely y =1
  EXPECT_NEAR(result.GetSolution(y), 1, tol);
  EXPECT_NEAR(result.GetSolution(z), 0, tol);
}

void MinimizeKLDivergence(const SolverInterface& solver, double tol) {
  // The probability p(x)
  const Eigen::Vector4d p(0.1, 0.2, 0.3, 0.4);

  MathematicalProgram prog;
  // q is the other probabilty distribution
  const auto q = prog.NewContinuousVariables<4>();
  // A valid probability should sum up to 1.
  prog.AddLinearEqualityConstraint(Eigen::RowVector4d::Ones(), 1, q);
  // A valid probability should be non-negative.
  prog.AddBoundingBoxConstraint(0, kInf, q);

  // prog minimizes the KL divergence KL(p || q) = ∑ₓ p(x) * log(p(x) / q(x)).
  // Equivalently, the KL divergence is ∑ₓ -p(x) * log(q(x) / p(x)).
  // We introduce a slack variable t(x), such that t(x) >= -p(x)* log(q(x) /
  // p(x)) namely (q(x), p(x), -t(x)) is in the exponential cone, and we
  // minimize ∑ₓ t(x).
  const auto t = prog.NewContinuousVariables<4>();
  for (int i = 0; i < 4; ++i) {
    prog.AddExponentialConeConstraint(
        Vector3<symbolic::Expression>(q(i), p(i), -t(i)));
  }
  prog.AddLinearCost(t.cast<symbolic::Expression>().sum());

  MathematicalProgramResult result;
  solver.Solve(prog, {}, {}, &result);
  EXPECT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(result.GetSolution(q), p, tol));
  EXPECT_TRUE(
      CompareMatrices(result.GetSolution(t), Eigen::Vector4d::Zero(), tol));
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
