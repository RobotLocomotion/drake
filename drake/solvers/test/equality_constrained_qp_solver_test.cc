#include "drake/solvers/equality_constrained_qp_solver.h"

namespace drake {
namespace solvers {
namespace {

// Tests the equality constrained convex QP solver, using example from
// [Nocedal 1999].

GTEST_TEST(TestEQP, Example) {
  // Quadratic objective matrix.
  Eigen::MatrixXd G(3,3);
  G << 6.0, 2.0, 1.0, 2.0, 5.0, 2.0, 1.0, 2.0, 4.0;

  // Linear objective vector.
  Eigen::VectorXd c(3);
  c << -8.0, -3.0, -3.0;

  // Linear constraint matrix
  Eigen::MatrixXd A(2,3);
  A << 1.0, 0.0, 1.0, 0.0, 1.0, 1.0;

  // Linear constraint vector
  Eigen::VectorXd b(2);
  b << 3.0, 0.0;

  // Setup the QP
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(3);
  auto objective = prog.AddQuadraticErrorCost(G, c);
  auto constraint = prog.AddLinearConstraint(A, b);

  // Solve the QP
  SolutionResult result = SolutionResult::kUnknownError;

  try {
    result = prog.Solve();
  }
  catch (const std::runtime_error& error) {
    if (std::string(error.what()).find("No solver available") !=
        std::string::npos)
      return;  // missing externals... the test should abort and report
  }

  EXPECT_EQ(result, SolutionResult::kSolutionFound);
}

}  // namespace ""
}  // namespace solvers
}  // namespace drake