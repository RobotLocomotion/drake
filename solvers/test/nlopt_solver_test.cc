#include "drake/solvers/nlopt_solver.h"

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/quadratic_program_examples.h"

namespace drake {
namespace solvers {
namespace test {
TEST_F(UnboundedLinearProgramTest0, TestNlopt) {
  prog_->SetInitialGuessForAllVariables(Eigen::Vector2d::Zero());
  NloptSolver solver;
  if (solver.available()) {
    MathematicalProgramResult result;
    solver.Solve(*prog_, {}, {}, &result);
    EXPECT_FALSE(result.is_success());
    EXPECT_EQ(result.get_optimal_cost(),
              -std::numeric_limits<double>::infinity());
    SolverOptions solver_options;
    solver_options.SetOption(solver.solver_id(), NloptSolver::MaxEvalName(), 1);
    solver.Solve(*prog_, {}, solver_options, &result);
    const int NLOPT_MAXEVAL_REACHED = 5;
    EXPECT_EQ(result.get_solver_details<NloptSolver>().status,
              NLOPT_MAXEVAL_REACHED);
  }
}

GTEST_TEST(QPtest, TestUnitBallExample) {
  NloptSolver solver;
  if (solver.available()) {
    TestQPonUnitBallExample(solver);
  }
}

GTEST_TEST(NloptSolverTest, TestNonconvexQP) {
  NloptSolver solver;
  if (solver.available()) {
    TestNonconvexQP(solver, false);
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
