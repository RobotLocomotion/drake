#include "drake/solvers/test/exponential_cone_program_examples.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

namespace drake {
namespace solvers {
namespace test {
const double kInf = std::numeric_limits<double>::infinity();

void MinimizeKLDivergence(const SolverInterface& solver, double tol) {
  // The probability p(x)
  const Eigen::Vector4d p(0.1, 0.2, 0.3, 0.4);

  MathematicalProgram prog1;
  // q is the other probabilty distribution
  const auto q = prog1.NewContinuousVariables<4>();
  // A valid probability should sum up to 1.
  prog1.AddLinearEqualityConstraint(Eigen::RowVector4d::Ones(), 1, q);
  // A valid probability should be non-negative.
  prog1.AddBoundingBoxConstraint(0, kInf, q);

  // prog1 minimizes the KL divergence KL(p || q) = ∑ₓ p(x)* log(p(x) / q(x)).
  // equivalently, the KL divergence is ∑ₓ -p(x)* log(q(x) / p(x))
  // we introduce a slack variable t(x), such that t(x) >= -p(x)* log(q(x) /
  // p(x)) namely (q(x), p(x), -t(x)) is in the exponential cone, and we
  // minimize ∑ₓ t(x).
  const auto t = prog1.NewContinuousVariables<4>();
  for (int i = 0; i < 4; ++i) {
    prog1.AddExponentialConeConstraint(
        Vector3<symbolic::Expression>(q(i), p(i), -t(i)));
  }
  prog1.AddLinearCost(t.cast<symbolic::Expression>().sum());

  MathematicalProgramResult result;
  solver.Solve(prog1, {}, {}, &result);
  EXPECT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(result.GetSolution(q), p, tol));
  EXPECT_TRUE(
      CompareMatrices(result.GetSolution(t), Eigen::Vector4d::Zero(), tol));
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
