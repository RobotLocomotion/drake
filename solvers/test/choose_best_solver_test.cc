#include "drake/solvers/choose_best_solver.h"

#include <gtest/gtest.h>

#include "drake/solvers/equality_constrained_qp_solver.h"
#include "drake/solvers/linear_system_solver.h"

namespace drake {
namespace solvers {
class ChooseBestSolverTest : public ::testing::Test {
 public:
  ChooseBestSolverTest() : prog_{}, x_{prog_.NewContinuousVariables<3>()} {}

  void CheckBestSolver(const SolverId& expected_solver_id) {
    std::unique_ptr<MathematicalProgramSolverInterface> solver =
        ChooseBestSolver(prog_);
    EXPECT_EQ(solver->solver_id(), expected_solver_id);
  }

 protected:
  MathematicalProgram prog_;
  VectorDecisionVariable<3> x_;
};

TEST_F(ChooseBestSolverTest, LinearSystemSolver) {
  prog_.AddLinearEqualityConstraint(x_(0) + x_(1), 1);
  CheckBestSolver(LinearSystemSolver::id());
}

TEST_F(ChooseBestSolverTest, EqualityConstrainedQPSolver) {
  prog_.AddQuadraticCost(x_(0) * x_(0));
  prog_.AddLinearEqualityConstraint(x_(0) + x_(1), 1);
  CheckBestSolver(EqualityConstrainedQPSolver::id());
}
}  // namespace solvers
}  // namespace drake
