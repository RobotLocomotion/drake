#include "drake/solvers/constraint.h"

#include <cmath>
#include <limits>
#include <set>
#include <unordered_map>

#include <fmt/format.h>

#include "drake/common/symbolic_decompose.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/matrix_util.h"

using std::abs;

namespace drake {
namespace solvers {

const double kInf = std::numeric_limits<double>::infinity();

namespace {
// Returns `True` if lb is -∞. Otherwise returns a symbolic formula `lb <= e`.
symbolic::Formula MakeLowerBound(const double lb,
                                 const symbolic::Expression& e) {
  if (lb == -std::numeric_limits<double>::infinity()) {
    return symbolic::Formula::True();
  } else {
    return lb <= e;
  }
}

// Returns `True` if ub is ∞. Otherwise returns a symbolic formula `e <= ub`.
symbolic::Formula MakeUpperBound(const symbolic::Expression& e,
                                 const double ub) {
  if (ub == std::numeric_limits<double>::infinity()) {
    return symbolic::Formula::True();
  } else {
    return e <= ub;
  }
}

std::ostream& DisplayConstraint(const Constraint& constraint, std::ostream& os,
                                const std::string& name,
                                const VectorX<symbolic::Variable>& vars,
                                bool equality) {
  os << name;
  VectorX<symbolic::Expression> e(constraint.num_constraints());
  constraint.Eval(vars, &e);
  // Append the description (when provided).
  const std::string& description = constraint.get_description();
  if (!description.empty()) {
    os << " described as '" << description << "'";
  }
  os << "\n";
  for (int i = 0; i < constraint.num_constraints(); ++i) {
    if (equality) {
      os << e(i) << " == " << constraint.upper_bound()(i) << "\n";
    } else {
      os << constraint.lower_bound()(i) << " <= " << e(i)
         << " <= " << constraint.upper_bound()(i) << "\n";
    }
  }
  return os;
}
}  // namespace

void Constraint::check(int num_constraints) const {
  if (lower_bound_.size() != num_constraints ||
      upper_bound_.size() != num_constraints) {
    throw std::invalid_argument(
        fmt::format("Constraint {} expects lower and upper bounds of size {}, "
                    "got lower bound of size {} and upper bound of size {}.",
                    this->get_description(), num_constraints,
                    lower_bound_.size(), upper_bound_.size()));
  }
}

symbolic::Formula Constraint::DoCheckSatisfied(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x) const {
  VectorX<symbolic::Expression> y(num_constraints());
  DoEval(x, &y);
  symbolic::Formula f{symbolic::Formula::True()};
  for (int i = 0; i < num_constraints(); ++i) {
    if (lower_bound_[i] == upper_bound_[i]) {
      // Add 'lbᵢ = yᵢ' to f.
      f = f && lower_bound_[i] == y[i];
    } else {
      // Add 'lbᵢ ≤ yᵢ ≤ ubᵢ' to f.
      f = f && MakeLowerBound(lower_bound_[i], y[i]) &&
          MakeUpperBound(y[i], upper_bound_[i]);
    }
  }
  return f;
}

template <typename DerivedX, typename ScalarY>
void QuadraticConstraint::DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                                        VectorX<ScalarY>* y) const {
  y->resize(num_constraints());
  *y = .5 * x.transpose() * Q_ * x + b_.transpose() * x;
}

void QuadraticConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                 Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void QuadraticConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                 AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}

void QuadraticConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  DoEvalGeneric(x, y);
}

std::ostream& QuadraticConstraint::DoDisplay(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  return DisplayConstraint(*this, os, "QuadraticConstraint", vars, false);
}

namespace {
int get_lorentz_cone_constraint_size(
    LorentzConeConstraint::EvalType eval_type) {
  return eval_type == LorentzConeConstraint::EvalType::kNonconvex ? 2 : 1;
}
}  // namespace

LorentzConeConstraint::LorentzConeConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b, EvalType eval_type)
    : Constraint(
          get_lorentz_cone_constraint_size(eval_type), A.cols(),
          Eigen::VectorXd::Zero(get_lorentz_cone_constraint_size(eval_type)),
          Eigen::VectorXd::Constant(get_lorentz_cone_constraint_size(eval_type),
                                    kInf)),
      A_(A.sparseView()),
      A_dense_(A),
      b_(b),
      eval_type_{eval_type} {
  DRAKE_DEMAND(A_.rows() >= 2);
  DRAKE_ASSERT(A_.rows() == b_.rows());
}

namespace {
template <typename DerivedX>
void LorentzConeConstraintEvalConvex2Autodiff(
    const Eigen::MatrixXd& A_dense, const Eigen::VectorXd& b,
    const Eigen::MatrixBase<DerivedX>& x, VectorX<AutoDiffXd>* y) {
  const Eigen::VectorXd x_val = math::ExtractValue(x);
  const Eigen::VectorXd z_val = A_dense * x_val + b;
  const double z_tail_squared_norm = z_val.tail(z_val.rows() - 1).squaredNorm();
  Vector1d y_val(z_val(0) - std::sqrt(z_tail_squared_norm));
  Eigen::RowVectorXd dy_dz(z_val.rows());
  dy_dz(0) = 1;
  // We use a small value epsilon to approximate the gradient of sqrt(z) as
  // ∂sqrt(zᵀz) / ∂z ≈ z(i) / sqrt(zᵀz + eps)
  const double eps = 1E-12;
  dy_dz.tail(z_val.rows() - 1) =
      -z_val.tail(z_val.rows() - 1) / std::sqrt(z_tail_squared_norm + eps);
  Eigen::RowVectorXd dy = dy_dz * A_dense * math::ExtractGradient(x);
  (*y) = math::InitializeAutoDiff(y_val, dy);
}
}  // namespace

template <typename DerivedX, typename ScalarY>
void LorentzConeConstraint::DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                                          VectorX<ScalarY>* y) const {
  using std::pow;

  const VectorX<ScalarY> z = A_dense_ * x.template cast<ScalarY>() + b_;
  y->resize(num_constraints());
  switch (eval_type_) {
    case EvalType::kConvex: {
      (*y)(0) = z(0) - z.tail(z.rows() - 1).norm();
      break;
    }
    case EvalType::kConvexSmooth: {
      if constexpr (std::is_same_v<ScalarY, AutoDiffXd>) {
        LorentzConeConstraintEvalConvex2Autodiff(A_dense_, b_, x, y);
      } else {
        (*y)(0) = z(0) - z.tail(z.rows() - 1).norm();
      }
      break;
    }
    case EvalType::kNonconvex: {
      (*y)(0) = z(0);
      (*y)(1) = pow(z(0), 2) - z.tail(z.size() - 1).squaredNorm();
      break;
    }
  }
}

void LorentzConeConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                   Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void LorentzConeConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                   AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}

void LorentzConeConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  DoEvalGeneric(x, y);
}

std::ostream& LorentzConeConstraint::DoDisplay(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  return DisplayConstraint(*this, os, "LorentzConeConstraint", vars, false);
}

template <typename DerivedX, typename ScalarY>
void RotatedLorentzConeConstraint::DoEvalGeneric(
    const Eigen::MatrixBase<DerivedX>& x, VectorX<ScalarY>* y) const {
  const VectorX<ScalarY> z = A_dense_ * x.template cast<ScalarY>() + b_;
  y->resize(num_constraints());
  (*y)(0) = z(0);
  (*y)(1) = z(1);
  (*y)(2) = z(0) * z(1) - z.tail(z.size() - 2).squaredNorm();
}

void RotatedLorentzConeConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void RotatedLorentzConeConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}

void RotatedLorentzConeConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  DoEvalGeneric(x, y);
}

std::ostream& RotatedLorentzConeConstraint::DoDisplay(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  return DisplayConstraint(*this, os, "RotatedLorentzConeConstraint", vars,
                           false);
}

template <typename DerivedX, typename ScalarY>
void LinearConstraint::DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                                     VectorX<ScalarY>* y) const {
  y->resize(num_constraints());
  (*y) = A_ * x.template cast<ScalarY>();
}

void LinearConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                              Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void LinearConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                              AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}

void LinearConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  DoEvalGeneric(x, y);
}

std::ostream& LinearConstraint::DoDisplay(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  return DisplayConstraint(*this, os, "LinearConstraint", vars, false);
}

std::ostream& LinearEqualityConstraint::DoDisplay(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  return DisplayConstraint(*this, os, "LinearEqualityConstraint", vars, true);
}

template <typename DerivedX, typename ScalarY>
void BoundingBoxConstraint::DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                                          VectorX<ScalarY>* y) const {
  y->resize(num_constraints());
  (*y) = x.template cast<ScalarY>();
}

void BoundingBoxConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                   Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}
void BoundingBoxConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                   AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}

void BoundingBoxConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  DoEvalGeneric(x, y);
}

std::ostream& BoundingBoxConstraint::DoDisplay(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  return DisplayConstraint(*this, os, "BoundingBoxConstraint", vars, false);
}

template <typename DerivedX, typename ScalarY>
void LinearComplementarityConstraint::DoEvalGeneric(
    const Eigen::MatrixBase<DerivedX>& x, VectorX<ScalarY>* y) const {
  y->resize(num_constraints());
  (*y) = (M_ * x.template cast<ScalarY>()) + q_;
}

void LinearComplementarityConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void LinearComplementarityConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}

void LinearComplementarityConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  DoEvalGeneric(x, y);
}

bool LinearComplementarityConstraint::DoCheckSatisfied(
    const Eigen::Ref<const Eigen::VectorXd>& x, const double tol) const {
  // Check: x >= 0 && Mx + q >= 0 && x'(Mx + q) == 0
  Eigen::VectorXd y(num_constraints());
  DoEval(x, &y);
  return (x.array() > -tol).all() && (y.array() > -tol).all() &&
         (abs(x.dot(y)) < tol);
}

bool LinearComplementarityConstraint::DoCheckSatisfied(
    const Eigen::Ref<const AutoDiffVecXd>& x, const double tol) const {
  AutoDiffVecXd y(num_constraints());
  DoEval(x, &y);
  return (x.array() > -tol).all() && (y.array() > -tol).all() &&
         (abs(x.dot(y)) < tol);
}

symbolic::Formula LinearComplementarityConstraint::DoCheckSatisfied(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x) const {
  VectorX<symbolic::Expression> y(num_constraints());
  DoEval(x, &y);

  symbolic::Formula f{symbolic::Formula::True()};
  // 1. Mx + q >= 0
  for (VectorX<symbolic::Expression>::Index i = 0; i < y.size(); ++i) {
    f = f && (y(i) >= 0);
  }
  // 2. x >= 0
  for (VectorX<symbolic::Expression>::Index i = 0; i < x.size(); ++i) {
    f = f && (x(i) >= 0);
  }
  // 3. x'(Mx + q) == 0
  f = f && (x.dot(y) == 0);
  return f;
}

void PositiveSemidefiniteConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DRAKE_ASSERT(x.rows() == num_constraints() * num_constraints());
  Eigen::MatrixXd S(num_constraints(), num_constraints());

  for (int i = 0; i < num_constraints(); ++i) {
    S.col(i) = x.segment(i * num_constraints(), num_constraints());
  }

  DRAKE_ASSERT(S.rows() == num_constraints());

  // This uses the lower diagonal part of S to compute the eigen values.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(S);
  *y = eigen_solver.eigenvalues();
}

void PositiveSemidefiniteConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>&, AutoDiffVecXd*) const {
  throw std::logic_error(
      "The Eval function for positive semidefinite constraint is not defined, "
      "since the eigen solver does not work for AutoDiffScalar.");
}

void PositiveSemidefiniteConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::logic_error(
      "The Eval function for positive semidefinite constraint is not defined, "
      "since the eigen solver does not work for symbolic::Expression.");
}

void LinearMatrixInequalityConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DRAKE_ASSERT(x.rows() == static_cast<int>(F_.size()) - 1);
  Eigen::MatrixXd S = F_[0];
  for (int i = 1; i < static_cast<int>(F_.size()); ++i) {
    S += x(i - 1) * F_[i];
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(S);
  *y = eigen_solver.eigenvalues();
}

void LinearMatrixInequalityConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>&, AutoDiffVecXd*) const {
  throw std::logic_error(
      "The Eval function for positive semidefinite constraint is not defined, "
      "since the eigen solver does not work for AutoDiffScalar.");
}

void LinearMatrixInequalityConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::logic_error(
      "The Eval function for positive semidefinite constraint is not defined, "
      "since the eigen solver does not work for symbolic::Expression.");
}

LinearMatrixInequalityConstraint::LinearMatrixInequalityConstraint(
    const std::vector<Eigen::Ref<const Eigen::MatrixXd>>& F,
    double symmetry_tolerance)
    : Constraint(F.empty() ? 0 : F.front().rows(),
                 F.empty() ? 0 : F.size() - 1),
      F_(F.begin(), F.end()),
      matrix_rows_(F.empty() ? 0 : F.front().rows()) {
  DRAKE_DEMAND(!F.empty());
  set_bounds(Eigen::VectorXd::Zero(matrix_rows_),
             Eigen::VectorXd::Constant(
                 matrix_rows_, std::numeric_limits<double>::infinity()));
  for (const auto& Fi : F) {
    DRAKE_ASSERT(Fi.rows() == matrix_rows_);
    DRAKE_ASSERT(math::IsSymmetric(Fi, symmetry_tolerance));
  }
}

ExpressionConstraint::ExpressionConstraint(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub)
    : Constraint(v.rows(), GetDistinctVariables(v).size(), lb, ub),
      expressions_(v) {
  // Setup map_var_to_index_ and vars_ so that
  //   map_var_to_index_[vars_(i).get_id()] = i.
  for (int i = 0; i < expressions_.size(); ++i) {
    symbolic::ExtractAndAppendVariablesFromExpression(expressions_(i), &vars_,
                                                      &map_var_to_index_);
  }

  derivatives_ = symbolic::Jacobian(expressions_, vars_);

  // Setup the environment.
  for (int i = 0; i < vars_.size(); i++) {
    environment_.insert(vars_[i], 0.0);
  }
}

void ExpressionConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                  Eigen::VectorXd* y) const {
  DRAKE_DEMAND(x.rows() == vars_.rows());

  // Set environment with current x values.
  for (int i = 0; i < vars_.size(); i++) {
    environment_[vars_[i]] = x(map_var_to_index_.at(vars_[i].get_id()));
  }

  // Evaluate into the output, y.
  y->resize(num_constraints());
  for (int i = 0; i < num_constraints(); i++) {
    (*y)[i] = expressions_[i].Evaluate(environment_);
  }
}

void ExpressionConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                  AutoDiffVecXd* y) const {
  DRAKE_DEMAND(x.rows() == vars_.rows());

  // Set environment with current x values.
  for (int i = 0; i < vars_.size(); i++) {
    environment_[vars_[i]] = x(map_var_to_index_.at(vars_[i].get_id())).value();
  }

  // Evaluate value and derivatives into the output, y.
  // Using ∂yᵢ/∂zⱼ = ∑ₖ ∂fᵢ/∂xₖ ∂xₖ/∂zⱼ.
  y->resize(num_constraints());
  Eigen::VectorXd dyidx(x.size());
  for (int i = 0; i < num_constraints(); i++) {
    (*y)[i].value() = expressions_[i].Evaluate(environment_);
    for (int k = 0; k < x.size(); k++) {
      dyidx[k] = derivatives_(i, k).Evaluate(environment_);
    }

    (*y)[i].derivatives().resize(x(0).derivatives().size());
    for (int j = 0; j < x(0).derivatives().size(); j++) {
      (*y)[i].derivatives()[j] = 0.0;
      for (int k = 0; k < x.size(); k++) {
        (*y)[i].derivatives()[j] += dyidx[k] * x(k).derivatives()[j];
      }
    }
  }
}

void ExpressionConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  DRAKE_DEMAND(x.rows() == vars_.rows());
  symbolic::Substitution subst;
  for (int i = 0; i < vars_.size(); ++i) {
    if (!vars_[i].equal_to(x[i])) {
      subst.emplace(vars_[i], x[i]);
    }
  }
  y->resize(num_constraints());
  if (subst.empty()) {
    *y = expressions_;
  } else {
    for (int i = 0; i < num_constraints(); ++i) {
      (*y)[i] = expressions_[i].Substitute(subst);
    }
  }
}

std::ostream& ExpressionConstraint::DoDisplay(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  return DisplayConstraint(*this, os, "ExpressionConstraint", vars, false);
}

ExponentialConeConstraint::ExponentialConeConstraint(
    const Eigen::Ref<const Eigen::SparseMatrix<double>>& A,
    const Eigen::Ref<const Eigen::Vector3d>& b)
    : Constraint(
          2, A.cols(), Eigen::Vector2d::Zero(),
          Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity())),
      A_{A},
      b_{b} {
  DRAKE_DEMAND(A.rows() == 3);
}

template <typename DerivedX, typename ScalarY>
void ExponentialConeConstraint::DoEvalGeneric(
    const Eigen::MatrixBase<DerivedX>& x, VectorX<ScalarY>* y) const {
  y->resize(num_constraints());
  Vector3<ScalarY> z = A_ * x.template cast<ScalarY>() + b_;
  using std::exp;
  (*y)(0) = z(0) - z(1) * exp(z(2) / z(1));
  (*y)(1) = z(1);
}

void ExponentialConeConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void ExponentialConeConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                       AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}

void ExponentialConeConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  DoEvalGeneric(x, y);
}
}  // namespace solvers
}  // namespace drake
