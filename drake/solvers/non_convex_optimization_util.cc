#include "drake/solvers/non_convex_optimization_util.h"

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
DecomposeNonConvexQuadraticForm(
    const Eigen::Ref<const Eigen::MatrixXd>& Q) {
  if (Q.rows() != Q.cols()) {
    throw std::runtime_error("Q is not a square matrix.");
  }
  // If Q is positive semidefinite, then Q1 = Q and Q2 = 0.
  // If Q is negative semidefinite, then Q1 = 0 and Q2 = -Q.
  const Eigen::MatrixXd Q_sym = (Q + Q.transpose()) / 2;
  Eigen::LDLT<Eigen::MatrixXd> ldlt(Q_sym);
  if (ldlt.info() == Eigen::Success) {
    if (ldlt.isPositive()) {
      Eigen::MatrixXd Q2 = Eigen::MatrixXd::Zero(Q.rows(), Q.rows());
      return std::make_pair(Q_sym, Q2);
    } else if (ldlt.isNegative()) {
      Eigen::MatrixXd Q1 = Eigen::MatrixXd::Zero(Q.rows(), Q.rows());
      return std::make_pair(Q1, -Q_sym);
    }
  }
  MathematicalProgram prog;
  auto Q1 = prog.NewSymmetricContinuousVariables(Q.rows(), "Q1");
  auto Q2 = prog.NewSymmetricContinuousVariables(Q.rows(), "Q2");
  symbolic::Variable s = prog.NewContinuousVariables<1>("s")(0);
  prog.AddPositiveSemidefiniteConstraint(Q1);
  prog.AddPositiveSemidefiniteConstraint(Q2);
  prog.AddLinearConstraint(Q1.array() - Q2.array() == Q_sym.array());
  prog.AddLinearConstraint(symbolic::Expression(s) >=
                           Q1.cast<symbolic::Expression>().trace());
  prog.AddLinearConstraint(symbolic::Expression(s) >=
                           Q2.cast<symbolic::Expression>().trace());
  prog.AddCost(s);
  SolutionResult result = prog.Solve();
  // This problem should always be feasible, since we can choose Q1 to a large
  // diagonal matrix, and Q2 will also have large diagonal entries. Both Q1 and
  // Q2 are diagonally dominant, thus they are both positive definite.
  // Due to positive definiteness, both trace(Q1) and trace(Q2) are
  // non-negative, so min(max(trace(Q1), trace(Q2)) is lower bounded. Hence,
  // this optimal cost is not un-bounded.
  DRAKE_DEMAND(result == SolutionResult::kSolutionFound);
  auto Q1_sol = prog.GetSolution(Q1);
  auto Q2_sol = prog.GetSolution(Q2);
  return std::make_pair(Q1_sol, Q2_sol);
}
}  // namespace solvers
}  // namespace drake
