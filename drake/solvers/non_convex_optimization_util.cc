#include "drake/solvers/non_convex_optimization_util.h"

#include "drake/math/quadratic_form.h"
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

Eigen::MatrixXd DecomposeYasXtransposeTimesX(const Eigen::Ref<const Eigen::MatrixXd>& Y, double psd_tol) {
  DRAKE_DEMAND(Y.rows() == Y.cols());
  DRAKE_DEMAND(psd_tol >= 0)
  Eigen::LLT<Eigen::MatrixXd> llt_Y(Y);
  if (llt_Y.info() == Eigen::Success) {
    return llt_Y.matrixU();
  } else {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_Y(Y);
    if (es_Y.info() == Eigen::Success) {
      Eigen::MatrixXd X(Y.rows(), Y.cols());
      int X_row_count = 0;
      for (int i = 0; i < es_Y.eigenvalues().rows(); ++i) {
        if (es_Y.eigenvalues()(i) < -psd_tol) {
          throw std::runtime_error("Y is not positive definite.");
        } else if (es_Y.eigenvalues()(i) > psd_tol) {
          X.row(X_row_count++) = std::sqrt(es_Y.eigenvalues()(i)) * es_Y.eigenvectors().col(i).transpose();
        }
      }
      return X.topRows(X_row_count);
    }
  }
  throw std::runtime_error("Y is not PSD.");
}

std::tuple<Binding<LinearConstraint>, Binding<RotatedLorentzConeConstraint>,
           Binding<RotatedLorentzConeConstraint>, VectorDecisionVariable<2>>
RelaxNonConvexQuadraticInequalityConstraintInTrustRegion(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& Q1,
    const Eigen::Ref<const Eigen::MatrixXd>& Q2,
    const Eigen::Ref<const Eigen::VectorXd>& p, double lower_bound,
    double upper_bound,
    const Eigen::Ref<const Eigen::VectorXd>& linearization_point,
    double trust_region_gap) {
  const auto& x0 = linearization_point;
  if (Q1.rows() != Q1.cols()) {
    throw std::runtime_error("Q1 should be a square matrix.");
  }
  if (Q2.rows() != Q2.cols()) {
    throw std::runtime_error("Q2 should be a square matrix.");
  }
  if (Q1.rows() != Q2.rows() || Q1.rows() != p.rows() ||
      Q1.rows() != x0.rows() || Q1.rows() != x.rows()) {
    throw std::runtime_error("The input matrices do not have the right size.");
  }
  if (trust_region_gap <= 0) {
    throw std::runtime_error("trust_region_gap should be positive.");
  }
  auto z = prog->NewContinuousVariables<2>("z");
  Binding<LinearConstraint> linear_constraint = prog->AddLinearConstraint(
      z(0) - z(1) + p.dot(x) <= upper_bound &&
      z(0) - z(1) + p.dot(x) >= lower_bound &&
      z(0) <= 2 * x0.dot(Q1 * x) - x0.dot(Q1 * x0) + trust_region_gap &&
      z(1) <= 2 * x0.dot(Q2 * x) - x0.dot(Q2 * x0) + trust_region_gap);
  double psd_tol = 1E-15;
  const Eigen::MatrixXd A1 = DecomposeYasXtransposeTimesX(Q1, psd_tol);
  const Eigen::MatrixXd A2 = DecomposeYasXtransposeTimesX(Q2, psd_tol);
  VectorX<symbolic::Expression> lorentz_cone_expr1(2 + A1.rows());
  VectorX<symbolic::Expression> lorentz_cone_expr2(2 + A2.rows());
  lorentz_cone_expr1 << 1, z(0), A1 * x;
  lorentz_cone_expr1 << 1, z(1), A2 * x;
  Binding<RotatedLorentzConeConstraint> lorentz_cone_constraint1 =
      prog->AddRotatedLorentzConeConstraint(lorentz_cone_expr1);
  Binding<RotatedLorentzConeConstraint> lorentz_cone_constraint2 =
      prog->AddRotatedLorentzConeConstraint(lorentz_cone_expr2);
  return std::make_tuple(linear_constraint, lorentz_cone_constraint1,
                         lorentz_cone_constraint2, z);
}
}  // namespace solvers
}  // namespace drake
