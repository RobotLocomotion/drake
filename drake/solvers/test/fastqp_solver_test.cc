#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace {

/// Solve a series of QPs with the objective being the Euclidean distance
/// from a point which moves along the unit circle (L2 ball), but
/// constrained to lie inside the L1 ball.  Implemented in 2D, so that the
/// active set moves along 4 faces of the L1 ball.

GTEST_TEST(testFastQP, unitBallExample) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(2);

  Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
  Eigen::Vector2d x_desired;
  x_desired << 1.0, 0.0;
  auto objective = prog.AddQuadraticErrorCost(Q, x_desired);

  Eigen::Matrix2d A;
  A << 1.0, 1.0, -1.0, 1.0;
  Eigen::Vector2d ub = Eigen::Vector2d::Constant(1.0);
  Eigen::Vector2d lb = Eigen::Vector2d::Constant(-1.0);
  auto constraint = prog.AddLinearConstraint(A, lb, ub);
  Eigen::Vector2d x_expected;

  const int N = 40;  // number of points to test around the circle
  for (int i = 0; i < N; i++) {
    double theta = 2.0 * M_PI * i / N;
    x_desired << sin(theta), cos(theta);
    objective->UpdateQuadraticAndLinearTerms(2.0 * Q, -2.0 * Q * x_desired);

    if (theta <= M_PI_2) {
      // simple lagrange multiplier problem:
      // min (x-x_d)^2 + (y-y_d)^2 s.t. x+y=1
      x_expected << (x_desired(0) - x_desired(1) + 1.0) / 2.0,
          (x_desired(1) - x_desired(0) + 1.0) / 2.0;
    } else if (theta <= M_PI) {
      // min (x-x_d)^2 + (y-y_d)^2 s.t. x-y=1
      x_expected << (x_desired(0) + x_desired(1) + 1.0) / 2.0,
          (x_desired(0) + x_desired(1) - 1.0) / 2.0;
    } else if (theta <= 3.0 * M_PI_2) {
      // min (x-x_d)^2 + (y-y_d)^2 s.t. x+y=-1
      x_expected << (x_desired(0) - x_desired(1) - 1.0) / 2.0,
          (x_desired(1) - x_desired(0) - 1.0) / 2.0;
    } else {
      // min (x-x_d)^2 + (y-y_d)^2 s.t. x-y=-1
      x_expected << (x_desired(0) + x_desired(1) - 1.0) / 2.0,
          (x_desired(0) + x_desired(1) + 1.0) / 2.0;
    }

    SolutionResult result = SolutionResult::kUnknownError;

    try {
      result = prog.Solve();  // TODO(russt) call fastQP solver
                              // explicitly
    } catch (const std::runtime_error& error) {
      if (std::string(error.what()).find("No solver available") !=
          std::string::npos)
        return;  // missing externals... the test should abort and report
                 // success
    }

    EXPECT_EQ(result, SolutionResult::kSolutionFound);
    // TODO(russt) assert that fastQP only falls back on the expected
    // iterations

    EXPECT_TRUE(CompareMatrices(x.value(), x_expected, 1e-4,
                                MatrixCompareType::absolute));
  }

  // provide some test coverage for changing Q
  //
  {
    // now 2(x-xd)^2 + (y-yd)^2 s.t. x+y=1
    x_desired << 1.0, 1.0;
    Q(0, 0) = 2.0;
    objective->UpdateQuadraticAndLinearTerms(2.0 * Q, -2.0 * Q * x_desired);

    x_expected << 2.0 / 3.0, 1.0 / 3.0;

    SolutionResult result = SolutionResult::kUnknownError;

    prog.SetSolverOption("GUROBI", "BarConvTol", 1E-9);
    ASSERT_NO_THROW(result = prog.Solve());
    EXPECT_EQ(result, SolutionResult::kSolutionFound);

    EXPECT_TRUE(CompareMatrices(x.value(), x_expected, 1e-5,
                                MatrixCompareType::absolute));
  }
}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake
