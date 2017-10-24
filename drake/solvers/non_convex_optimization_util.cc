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

std::shared_ptr<RotatedLorentzConeConstraint>
RelaxNonConvexQuadraticInequalityConstraintInTrustRegion(
    const Eigen::Ref<const Eigen::MatrixXd>& Q1,
    const Eigen::Ref<const Eigen::MatrixXd>& Q2,
    const Eigen::Ref<const Eigen::VectorXd>& p, double upper_bound,
    const Eigen::Ref<const Eigen::VectorXd>& linearization_point,
    double trust_region_gap) {
  if (Q1.rows() != Q1.cols()) {
    throw std::runtime_error("Q1 should be a square matrix.");
  }
  if (Q2.rows() != Q2.cols()) {
    throw std::runtime_error("Q2 should be a square matrix.");
  }
  if (Q1.rows() != Q2.rows() || Q1.rows() != p.rows() || Q1.rows() != linearization_point.rows()) {
    throw std::runtime_error("The input matrices do not have the right size.");
  }
  Eigen::LLT<Eigen::MatrixXd> llt_Q2(Q2);
  if (llt_Q2.info() != Eigen::Success) {
    throw std::runtime_error("Q2 should be positive definite.");
  }
  if (trust_region_gap <= 0) {
    throw std::runtime_error("trust_region_gap should be positive.");
  }
  // The relaxed constraint xᵀQ₁x <= r + 2 x₀ᵀQ₂(x - x₀) + x₀ᵀQ₂x₀ + d - pᵀx
  // can be written as Ax+b in the rotated Lorentz cone
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 + Q1.rows(), Q1.rows());
  Eigen::VectorXd b = Eigen::VectorXd::Zero(2 + Q1.rows());
  const auto& x0 = linearization_point;
  A.row(0) = 2 * x0.transpose() * Q2 - p.transpose();
  b(0) = upper_bound - x0.dot(Q2 * x0) + trust_region_gap;
  b(1) = 1;
  Eigen::LLT<Eigen::MatrixXd> llt_Q1(Q1);
  if (llt_Q1.info() != Eigen::Success) {
    throw std::runtime_error("Q1 should be positive definite.");
  }
  A.bottomRows(Q1.rows()) = llt_Q1.matrixU();
  return std::make_shared<RotatedLorentzConeConstraint>(A, b);
}
}  // namespace solvers
}  // namespace drake
