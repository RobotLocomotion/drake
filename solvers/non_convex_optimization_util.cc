#include "drake/solvers/non_convex_optimization_util.h"

#include <limits>

#include "drake/math/quadratic_form.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace solvers {
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> DecomposeNonConvexQuadraticForm(
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
  const MathematicalProgramResult result = Solve(prog);
  // This problem should always be feasible, since we can choose Q1 to a large
  // diagonal matrix, and Q2 will also have large diagonal entries. Both Q1 and
  // Q2 are diagonally dominant, thus they are both positive definite.
  // Due to positive definiteness, both trace(Q1) and trace(Q2) are
  // non-negative, so min(max(trace(Q1), trace(Q2)) is lower bounded. Hence,
  // this optimal cost is not un-bounded.
  DRAKE_DEMAND(result.is_success());
  auto Q1_sol = result.GetSolution(Q1);
  auto Q2_sol = result.GetSolution(Q2);
  return std::make_pair(Q1_sol, Q2_sol);
}

std::tuple<Binding<LinearConstraint>,
           std::vector<Binding<RotatedLorentzConeConstraint>>,
           VectorXDecisionVariable>
AddRelaxNonConvexQuadraticConstraintInTrustRegion(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& Q1,
    const Eigen::Ref<const Eigen::MatrixXd>& Q2,
    const Eigen::Ref<const VectorXDecisionVariable>& y,
    const Eigen::Ref<const Eigen::VectorXd>& p,
    double lower_bound,
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
  if (Q1.rows() != Q2.rows() || Q1.rows() != x0.rows() ||
      Q1.rows() != x.rows() || p.rows() != y.rows()) {
    throw std::runtime_error(
        "The dimensions of the inputs are not consistent.");
  }
  if (trust_region_gap <= 0) {
    throw std::runtime_error("trust_region_gap should be positive.");
  }
  if (lower_bound > upper_bound) {
    throw std::runtime_error(
        "lower_bound should be no larger than upper_bound");
  }

  const bool Q1_is_zero = (Q1.array() == 0).all();
  const bool Q2_is_zero = (Q2.array() == 0).all();
  // Both Q1 and Q2 are zero.
  if (Q1_is_zero && Q2_is_zero) {
    throw std::runtime_error(
        "Both Q1 and Q2 are zero. The constraint is linear. The user should "
        "not call this function to relax a linear constraint.");
  }

  // Only Q1 is zero.
  const double psd_tol = 1E-15;
  if (Q1_is_zero && std::isinf(upper_bound)) {
    throw std::runtime_error(
        "Q1 is zero, and upper_bound is infinity. The constraint is convex. "
        "The user should not call this function to relax a convex "
        "constraint.");
  }
  if (Q2_is_zero && std::isinf(lower_bound)) {
    throw std::runtime_error(
        "Q2 is zero, and lower_bound is -infinity. The constraint is convex. "
        "The user should not call this function to relax a convex constraint.");
  }

  if (Q1_is_zero || Q2_is_zero) {
    const double z_coeff = Q1_is_zero ? -1 : 1;
    const Eigen::Matrix2d nonzero_Q = Q1_is_zero ? Q2 : Q1;
    auto z = prog->NewContinuousVariables<1>("z");
    Binding<RotatedLorentzConeConstraint> lorentz_cone1 =
        prog->AddRotatedLorentzConeConstraint(z(0), 1, x.dot(nonzero_Q * x),
                                              psd_tol);
    Eigen::Vector2d linear_lb(x0.dot(nonzero_Q * x0) - trust_region_gap,
                              lower_bound);
    Eigen::Vector2d linear_ub(std::numeric_limits<double>::infinity(),
                              upper_bound);
    Vector2<symbolic::Expression> linear_expr(2 * x0.dot(nonzero_Q * x) - z(0),
                                              p.dot(y) + z_coeff * z(0));
    Binding<LinearConstraint> linear_constraint =
        prog->AddConstraint(internal::BindingDynamicCast<LinearConstraint>(
            internal::ParseConstraint(linear_expr, linear_lb, linear_ub)));
    std::vector<Binding<RotatedLorentzConeConstraint>> lorentz_cones{
        {lorentz_cone1}};
    return std::make_tuple(linear_constraint, lorentz_cones, z);
  }

  // Neither Q1 nor Q2 is zero.
  auto z = prog->NewContinuousVariables<2>("z");
  Vector3<symbolic::Expression> linear_expressions;
  linear_expressions << z(0) - z(1) + p.dot(y), z(0) - 2 * x0.dot(Q1 * x),
      z(1) - 2 * x0.dot(Q2 * x);
  const Eigen::Vector3d linear_lb(lower_bound,
                                  -std::numeric_limits<double>::infinity(),
                                  -std::numeric_limits<double>::infinity());
  const Eigen::Vector3d linear_ub(upper_bound,
                                  -x0.dot(Q1 * x0) + trust_region_gap,
                                  -x0.dot(Q2 * x0) + trust_region_gap);
  Binding<LinearConstraint> linear_constraint =
      prog->AddConstraint(internal::BindingDynamicCast<LinearConstraint>(
          internal::ParseConstraint(linear_expressions, linear_lb, linear_ub)));

  const Eigen::MatrixXd A1 =
      math::DecomposePSDmatrixIntoXtransposeTimesX(Q1, psd_tol);
  const Eigen::MatrixXd A2 =
      math::DecomposePSDmatrixIntoXtransposeTimesX(Q2, psd_tol);
  VectorX<symbolic::Expression> lorentz_cone_expr1(2 + A1.rows());
  VectorX<symbolic::Expression> lorentz_cone_expr2(2 + A2.rows());
  lorentz_cone_expr1 << 1, z(0), A1 * x;
  lorentz_cone_expr2 << 1, z(1), A2 * x;
  Binding<RotatedLorentzConeConstraint> lorentz_cone_constraint1 =
      prog->AddRotatedLorentzConeConstraint(lorentz_cone_expr1);
  Binding<RotatedLorentzConeConstraint> lorentz_cone_constraint2 =
      prog->AddRotatedLorentzConeConstraint(lorentz_cone_expr2);
  std::vector<Binding<RotatedLorentzConeConstraint>> lorentz_cones{
      {lorentz_cone_constraint1, lorentz_cone_constraint2}};
  return std::make_tuple(linear_constraint, lorentz_cones, z);
}
}  // namespace solvers
}  // namespace drake
