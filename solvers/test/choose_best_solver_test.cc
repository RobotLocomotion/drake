#include "drake/solvers/choose_best_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
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

TEST_F(ChooseBestSolverTest, NoAvailableSolver) {
  // We don't have a solver for problem with both linear complementarity
  // constraint and binary variables.
  prog_.AddLinearComplementarityConstraint(
      Eigen::Matrix2d::Identity(), Eigen::Vector2d::Ones(), x_.tail<2>());
  prog_.NewBinaryVariables<2>();
  DRAKE_EXPECT_THROWS_MESSAGE(
      ChooseBestSolver(prog_), std::invalid_argument,
      "There is no available solver for the optimization program");
}
}  // namespace solvers
}  // namespace drake
