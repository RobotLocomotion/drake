#include "drake/solvers/constraint.h"

#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <unordered_map>

#include <fmt/format.h>

#include "drake/common/symbolic/decompose.h"
#include "drake/common/symbolic/latex.h"
#include "drake/common/text_logging.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/matrix_util.h"

using std::abs;

namespace drake {
namespace solvers {

using symbolic::ToLatex;
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

std::string ToLatexLowerBound(const Constraint& constraint, int precision) {
  const auto& lb = constraint.lower_bound();
  if (lb.array().isInf().all()) {
    return "";
  }
  if (lb == constraint.upper_bound()) {
    // We'll write this as == instead of <=.
    return "";
  }
  if (lb.size() == 1) {
    return ToLatex(lb[0], precision) + " \\le ";
  }
  return ToLatex(lb) + " \\le ";
}

std::string ToLatexUpperBound(const Constraint& constraint, int precision) {
  const auto& ub = constraint.upper_bound();
  if (ub.array().isInf().all()) {
    return "";
  }
  std::string comparator = (ub == constraint.lower_bound()) ? " = " : " \\le ";
  if (ub.size() == 1) {
    return comparator + ToLatex(ub[0], precision);
  }
  return comparator + ToLatex(ub);
}

std::string ToLatexConstraint(const Constraint& constraint,
                              const VectorX<symbolic::Variable>& vars,
                              int precision) {
  VectorX<symbolic::Expression> y(constraint.num_constraints());
  constraint.Eval(vars, &y);
  return fmt::format("{}{}{}", ToLatexLowerBound(constraint, precision),
                     constraint.num_constraints() == 1
                         ? symbolic::ToLatex(y[0], precision)
                         : symbolic::ToLatex(y, precision),
                     ToLatexUpperBound(constraint, precision));
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

QuadraticConstraint::~QuadraticConstraint() = default;

bool QuadraticConstraint::is_convex() const {
  switch (hessian_type_) {
    case HessianType::kZero:
      return true;
    case HessianType::kPositiveSemidefinite: {
      // lower_bound is -inf
      return std::isinf(lower_bound()(0)) && lower_bound()(0) < 0;
    }
    case HessianType::kNegativeSemidefinite: {
      // upper bound is inf
      return std::isinf(upper_bound()(0)) && upper_bound()(0) > 0;
    }
    case HessianType::kIndefinite: {
      return false;
    }
  }
  DRAKE_UNREACHABLE();
}

void QuadraticConstraint::UpdateHessianType(
    std::optional<HessianType> hessian_type) {
  if (hessian_type.has_value()) {
    hessian_type_ = hessian_type.value();
    return;
  }
  if (Q_.isZero(/*precision=*/0)) {
    hessian_type_ = HessianType::kZero;
    return;
  }
  if (Q_.trace() == 0) {
    // trace(Q_) is the summation of the eigen values. Since all eigen values
    // are real, then some eigen values are positive while some other eigen
    // values are negative (except the special case when all eigen values are
    // 0). We conclude that Q_ is indefinite (since we have checked zero matrix
    // already).
    hessian_type_ = HessianType::kIndefinite;
    return;
  }
  Eigen::LDLT<Eigen::MatrixXd> ldlt_solver;
  ldlt_solver.compute(Q_);
  if (ldlt_solver.info() != Eigen::Success) {
    // Fall back to an indefinite Hessian type if we cannot determine the
    // Hessian type.
    drake::log()->warn(
        "UpdateHessianType(): Unable to determine Hessian type of the "
        "Quadratic Constraint. Falling back to indefinite Hessian type. To get "
        "rid of this warning, if you know the type of the hessian (positive "
        "semidefinite, negative semidefinite, or indefinite), then set "
        "hessian_type explicitly when you construct "
        "or set the quadratic constraint, such as in"
        "QuadraticConstraint(), UpdateCoefficients() or "
        "AddQuadraticConstraint() functions.");
    hessian_type_ = HessianType::kIndefinite;
  } else {
    if (ldlt_solver.isPositive()) {
      hessian_type_ = HessianType::kPositiveSemidefinite;
    } else if (ldlt_solver.isNegative()) {
      hessian_type_ = HessianType::kNegativeSemidefinite;
    } else {
      hessian_type_ = HessianType::kIndefinite;
    }
  }
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

std::string QuadraticConstraint::DoToLatex(
    const VectorX<symbolic::Variable>& vars, int precision) const {
  // TODO(russt): Handle Q_ = c*I as a special case.
  std::string b_term;
  if (!b_.isZero()) {
    b_term = " + " + symbolic::ToLatex(b_.dot(vars), precision);
  }
  return fmt::format(
      "{}{}^T {} {}{}{}", ToLatexLowerBound(*this, precision),
      symbolic::ToLatex(vars), symbolic::ToLatex((0.5 * Q_).eval(), precision),
      symbolic::ToLatex(vars), b_term, ToLatexUpperBound(*this, precision));
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
  set_is_thread_safe(true);
  DRAKE_THROW_UNLESS(A_.rows() >= 2);
  DRAKE_THROW_UNLESS(A_.rows() == b_.rows());

  // Set the gradient sparsity pattern.
  UpdateGradientSparsityPattern();
}

LorentzConeConstraint::~LorentzConeConstraint() = default;

void LorentzConeConstraint::UpdateCoefficients(
    const Eigen::Ref<const Eigen::MatrixXd>& new_A,
    const Eigen::Ref<const Eigen::VectorXd>& new_b) {
  if (new_A.cols() != num_vars()) {
    throw std::invalid_argument(
        fmt::format("LorentzConeConstraint::UpdateCoefficients uses new_A with "
                    "{} columns to update a constraint with {} variables.",
                    new_A.cols(), num_vars()));
  }
  A_ = new_A.sparseView();
  A_dense_ = new_A;
  b_ = new_b;
  DRAKE_THROW_UNLESS(A_.rows() >= 2);
  DRAKE_THROW_UNLESS(A_.rows() == b_.rows());
  // Note that we don't need to update the lower and upper bounds as the
  // constraints lower/upper bounds are fixed (independent of A and b). The
  // bounds only depend on EvalType. When EvalType=kNonconvex, the lower/upper
  // bound is [0, 0]/[inf, inf] respectively; otherwise, the lower/upper bound
  // is 0/inf respectively.

  // Update the gradient sparsity pattern.
  UpdateGradientSparsityPattern();
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

std::string LorentzConeConstraint::DoToLatex(
    const VectorX<symbolic::Variable>& vars, int precision) const {
  VectorX<symbolic::Expression> z = A_ * vars + b_;
  return fmt::format("\\left|{}\\right|_2 \\le {}",
                     symbolic::ToLatex(z.tail(z.size() - 1).eval(), precision),
                     symbolic::ToLatex(z(0), precision));
}

void LorentzConeConstraint::UpdateGradientSparsityPattern() {
  std::vector<std::pair<int, int>> gradient_sparsity_pattern;
  switch (eval_type_) {
    case EvalType::kConvex:
    case EvalType::kConvexSmooth: {
      // For both kConvex and kConvexSmooth, we evaluate
      // y=z(0) - sqrt(z(1)²+z(2)²+...+z(n-1)²) where z = A*x+b
      // So dy/dx(i) is non-zero if A.col(i) is non-zero.
      for (int i = 0; i < A_.outerSize(); ++i) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(A_, i); it; ++it) {
          if (it.value() != 0) {
            gradient_sparsity_pattern.emplace_back(0, i);
            break;
          }
        }
      }
      break;
    }
    case EvalType::kNonconvex: {
      // We evaluate
      // y(0) = z(0)
      // y(1) = z(0)² -(z(1)²+...+z(n-1)²)
      // z = A*x+b.
      // so dy(0)/dx(i) is non-zero if A(0, i) is non-zero.
      // dy(1)/dx(i) is non-zero if A.col(i) is non-zero.
      for (int i = 0; i < A_.outerSize(); ++i) {
        bool is_column_zero = true;
        for (Eigen::SparseMatrix<double>::InnerIterator it(A_, i); it; ++it) {
          if (it.value() != 0) {
            is_column_zero = false;
            if (it.row() == 0) {
              gradient_sparsity_pattern.emplace_back(0, i);
            }
          }
        }
        if (!is_column_zero) {
          gradient_sparsity_pattern.emplace_back(1, i);
        }
      }
    }
  }
  this->SetGradientSparsityPattern(gradient_sparsity_pattern);
}

RotatedLorentzConeConstraint::RotatedLorentzConeConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b)
    : Constraint(
          3, A.cols(), Eigen::Vector3d::Constant(0.0),
          Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity())),
      A_(A.sparseView()),
      A_dense_(A),
      b_(b) {
  DRAKE_THROW_UNLESS(A_.rows() >= 3);
  DRAKE_THROW_UNLESS(A_.rows() == b_.rows());
  set_is_thread_safe(true);
  UpdateGradientSparsityPattern();
}

RotatedLorentzConeConstraint::~RotatedLorentzConeConstraint() = default;

void RotatedLorentzConeConstraint::UpdateCoefficients(
    const Eigen::Ref<const Eigen::MatrixXd>& new_A,
    const Eigen::Ref<const Eigen::VectorXd>& new_b) {
  if (new_A.cols() != num_vars()) {
    throw std::invalid_argument(fmt::format(
        "RotatedLorentzConeConstraint::UpdateCoefficients uses new_A with "
        "{} columns to update a constraint with {} variables.",
        new_A.cols(), num_vars()));
  }
  A_ = new_A.sparseView();
  A_dense_ = new_A;
  b_ = new_b;
  DRAKE_THROW_UNLESS(A_.rows() >= 3);
  DRAKE_THROW_UNLESS(A_.rows() == b_.rows());
  // Note that we don't need to update the lower and upper bounds as the
  // constraints lower/upper bounds are fixed (independent of A and b).

  UpdateGradientSparsityPattern();
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

std::string RotatedLorentzConeConstraint::DoToLatex(
    const VectorX<symbolic::Variable>& vars, int precision) const {
  VectorX<symbolic::Expression> z = A_ * vars + b_;
  return fmt::format(
      "0 \\le {},\\\\ 0 \\le {},\\\\ \\left|{}\\right|_2^2 \\le {}",
      symbolic::ToLatex(z(0), precision), symbolic::ToLatex(z(1), precision),
      symbolic::ToLatex(z.tail(z.size() - 2).eval(), precision),
      symbolic::ToLatex(z(0) * z(1), precision));
}

void RotatedLorentzConeConstraint::UpdateGradientSparsityPattern() {
  // Set the sparsity pattern.
  // We evaluate
  // y(0) = z(0)
  // y(1) = z(1)
  // y(2) = z(0)*z(1)-z(2)²-...-z(n-1)²
  // where z = A*x+b.
  // dy(0)/dx(i) is non-zero if A(0, i) is non-zero
  // dy(1)/dx(i) is non-zero if A(1, i) is non-zero
  // dy(2)/dx(i) is non-zero if A.col(i) is non-zero
  std::vector<std::pair<int, int>> gradient_sparsity_pattern;
  for (int i = 0; i < A_.outerSize(); ++i) {
    bool is_column_zero = true;
    for (Eigen::SparseMatrix<double>::InnerIterator it(A_, i); it; ++it) {
      if (it.value() != 0) {
        is_column_zero = false;
        if (it.row() == 0) {
          gradient_sparsity_pattern.emplace_back(0, i);
        } else if (it.row() == 1) {
          gradient_sparsity_pattern.emplace_back(1, i);
        }
      }
    }
    if (!is_column_zero) {
      gradient_sparsity_pattern.emplace_back(2, i);
    }
  }
  this->SetGradientSparsityPattern(gradient_sparsity_pattern);
}

PolynomialConstraint::~PolynomialConstraint() = default;

LinearConstraint::LinearConstraint(const Eigen::Ref<const Eigen::MatrixXd>& A,
                                   const Eigen::Ref<const Eigen::VectorXd>& lb,
                                   const Eigen::Ref<const Eigen::VectorXd>& ub)
    : Constraint(A.rows(), A.cols(), lb, ub), A_(A) {
  set_is_thread_safe(true);
  DRAKE_THROW_UNLESS(A.rows() == lb.rows());
  DRAKE_THROW_UNLESS(A.array().allFinite());
}

LinearConstraint::LinearConstraint(const Eigen::SparseMatrix<double>& A,
                                   const Eigen::Ref<const Eigen::VectorXd>& lb,
                                   const Eigen::Ref<const Eigen::VectorXd>& ub)
    : Constraint(A.rows(), A.cols(), lb, ub), A_(A) {
  set_is_thread_safe(true);
  DRAKE_THROW_UNLESS(A.rows() == lb.rows());
  DRAKE_THROW_UNLESS(A_.IsFinite());
}

LinearConstraint::~LinearConstraint() = default;

const Eigen::MatrixXd& LinearConstraint::GetDenseA() const {
  return A_.GetAsDense();
}

void LinearConstraint::UpdateCoefficients(
    const Eigen::Ref<const Eigen::MatrixXd>& new_A,
    const Eigen::Ref<const Eigen::VectorXd>& new_lb,
    const Eigen::Ref<const Eigen::VectorXd>& new_ub) {
  if (new_A.rows() != new_lb.rows() || new_lb.rows() != new_ub.rows()) {
    throw std::runtime_error("New constraints have invalid dimensions");
  }

  if (new_A.cols() != A_.get_as_sparse().cols()) {
    throw std::runtime_error("Can't change the number of decision variables");
  }

  A_ = new_A;
  DRAKE_THROW_UNLESS(A_.IsFinite());
  set_num_outputs(A_.get_as_sparse().rows());
  set_bounds(new_lb, new_ub);
}

void LinearConstraint::UpdateCoefficients(
    const Eigen::SparseMatrix<double>& new_A,
    const Eigen::Ref<const Eigen::VectorXd>& new_lb,
    const Eigen::Ref<const Eigen::VectorXd>& new_ub) {
  if (new_A.rows() != new_lb.rows() || new_lb.rows() != new_ub.rows()) {
    throw std::runtime_error("New constraints have invalid dimensions");
  }

  if (new_A.cols() != A_.get_as_sparse().cols()) {
    throw std::runtime_error("Can't change the number of decision variables");
  }
  A_ = new_A;
  DRAKE_THROW_UNLESS(A_.IsFinite());
  set_num_outputs(A_.get_as_sparse().rows());
  set_bounds(new_lb, new_ub);
}

void LinearConstraint::RemoveTinyCoefficient(double tol) {
  if (tol < 0) {
    throw std::invalid_argument(
        "RemoveTinyCoefficient: tol should be non-negative");
  }
  std::vector<Eigen::Triplet<double>> new_A_triplets;
  const auto& A_sparse = A_.get_as_sparse();
  new_A_triplets.reserve(A_sparse.nonZeros());
  for (int i = 0; i < A_sparse.outerSize(); ++i) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(A_sparse, i); it; ++it) {
      if (std::abs(it.value()) > tol) {
        new_A_triplets.emplace_back(it.row(), it.col(), it.value());
      }
    }
  }
  Eigen::SparseMatrix<double> new_A(A_sparse.rows(), A_sparse.cols());
  new_A.setFromTriplets(new_A_triplets.begin(), new_A_triplets.end());
  UpdateCoefficients(new_A, lower_bound(), upper_bound());
}

bool LinearConstraint::is_dense_A_constructed() const {
  return A_.is_dense_constructed();
}

template <typename DerivedX, typename ScalarY>
void LinearConstraint::DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                                     VectorX<ScalarY>* y) const {
  y->resize(num_constraints());
  (*y) = A_.get_as_sparse() * x.template cast<ScalarY>();
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

std::string LinearConstraint::DoToLatex(const VectorX<symbolic::Variable>& vars,
                                        int precision) const {
  if (num_constraints() == 1) {
    return fmt::format(
        "{}{}{}", ToLatexLowerBound(*this, precision),
        symbolic::ToLatex((A_.get_as_sparse() * vars)[0], precision),
        ToLatexUpperBound(*this, precision));
  }
  return fmt::format("{}{} {}{}", ToLatexLowerBound(*this, precision),
                     symbolic::ToLatex(GetDenseA(), precision),
                     symbolic::ToLatex(vars),
                     ToLatexUpperBound(*this, precision));
}

std::ostream& LinearEqualityConstraint::DoDisplay(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  return DisplayConstraint(*this, os, "LinearEqualityConstraint", vars, true);
}

namespace {
Eigen::SparseMatrix<double> ConstructSparseIdentity(int rows) {
  Eigen::SparseMatrix<double> mat(rows, rows);
  mat.setIdentity();
  return mat;
}
}  // namespace

BoundingBoxConstraint::BoundingBoxConstraint(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub)
    : LinearConstraint(ConstructSparseIdentity(lb.rows()), lb, ub) {}

BoundingBoxConstraint::~BoundingBoxConstraint() = default;

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

std::string BoundingBoxConstraint::DoToLatex(
    const VectorX<symbolic::Variable>& vars, int precision) const {
  return fmt::format("{}{}{}", ToLatexLowerBound(*this, precision),
                     num_constraints() == 1 ? symbolic::ToLatex(vars[0])
                                            : symbolic::ToLatex(vars),
                     ToLatexUpperBound(*this, precision));
}

LinearComplementarityConstraint::~LinearComplementarityConstraint() = default;

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

std::string LinearComplementarityConstraint::DoToLatex(
    const VectorX<symbolic::Variable>& vars, int precision) const {
  return fmt::format("0 \\le {} \\perp {} \\ge 0",
                     symbolic::ToLatex(vars, precision),
                     symbolic::ToLatex((M_ * vars + q_).eval(), precision));
}

PositiveSemidefiniteConstraint::PositiveSemidefiniteConstraint(int rows)
    : Constraint(rows, rows * rows, Eigen::VectorXd::Zero(rows),
                 Eigen::VectorXd::Constant(
                     rows, std::numeric_limits<double>::infinity())),
      matrix_rows_(rows) {
  set_is_thread_safe(true);
}

PositiveSemidefiniteConstraint::~PositiveSemidefiniteConstraint() = default;

void PositiveSemidefiniteConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DRAKE_THROW_UNLESS(x.rows() == num_constraints() * num_constraints());
  Eigen::MatrixXd S(num_constraints(), num_constraints());

  for (int i = 0; i < num_constraints(); ++i) {
    S.col(i) = x.segment(i * num_constraints(), num_constraints());
  }

  DRAKE_THROW_UNLESS(S.rows() == num_constraints());

  // This uses the lower diagonal part of S to compute the eigen values.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(S);
  *y = eigen_solver.eigenvalues();
}

void PositiveSemidefiniteConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>&, AutoDiffVecXd*) const {
  throw std::logic_error(
      "The Eval function for positive semidefinite constraint is not defined, "
      "since the eigen solver does not work for AutoDiffXd.");
}

void PositiveSemidefiniteConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::logic_error(
      "The Eval function for positive semidefinite constraint is not defined, "
      "since the eigen solver does not work for symbolic::Expression.");
}

std::string PositiveSemidefiniteConstraint::DoToLatex(
    const VectorX<symbolic::Variable>& vars, int precision) const {
  Eigen::Map<const MatrixX<symbolic::Variable>> S(vars.data(), matrix_rows(),
                                                  matrix_rows());
  return fmt::format("{} \\succeq 0", symbolic::ToLatex(S.eval(), precision));
}

void PositiveSemidefiniteConstraint::WarnOnSmallMatrixSize() const {
  if (matrix_rows_ == 1) {
    drake::log()->warn(
        "PositiveSemidefiniteConstraint: rows==1, please consider "
        "reformulating this as a linear inequality constraint for better "
        "speed/numerics.");
  } else if (matrix_rows_ == 2) {
    drake::log()->warn(
        "PositiveSemidefiniteConstraint: rows==2, please consider "
        "reformulating this as a rotated Lorentz cone constraint for better "
        "speed/numerics.");
  }
}

LinearMatrixInequalityConstraint::~LinearMatrixInequalityConstraint() = default;

void LinearMatrixInequalityConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DRAKE_THROW_UNLESS(x.rows() == static_cast<int>(F_.size()) - 1);
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
      "since the eigen solver does not work for AutoDiffXd.");
}

void LinearMatrixInequalityConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::logic_error(
      "The Eval function for positive semidefinite constraint is not defined, "
      "since the eigen solver does not work for symbolic::Expression.");
}

LinearMatrixInequalityConstraint::LinearMatrixInequalityConstraint(
    std::vector<Eigen::MatrixXd> F, double symmetry_tolerance)
    : Constraint(F.empty() ? 0 : F.front().rows(),
                 F.empty() ? 0 : F.size() - 1),
      F_{std::move(F)},
      matrix_rows_(F_.empty() ? 0 : F_.front().rows()) {
  DRAKE_THROW_UNLESS(!F_.empty());

  set_bounds(Eigen::VectorXd::Zero(matrix_rows_),
             Eigen::VectorXd::Constant(
                 matrix_rows_, std::numeric_limits<double>::infinity()));
  for (const auto& Fi : F_) {
    DRAKE_THROW_UNLESS(Fi.rows() == matrix_rows_);
    DRAKE_THROW_UNLESS(math::IsSymmetric(Fi, symmetry_tolerance));
  }
  set_is_thread_safe(true);
}

std::string LinearMatrixInequalityConstraint::DoToLatex(
    const VectorX<symbolic::Variable>& vars, int precision) const {
  MatrixX<symbolic::Expression> S = F_[0];
  for (int i = 1; i < static_cast<int>(F_.size()); ++i) {
    S += vars(i - 1) * F_[i];
  }
  return fmt::format("{} \\succeq 0", symbolic::ToLatex(S, precision));
}

void LinearMatrixInequalityConstraint::WarnOnSmallMatrixSize() const {
  // TODO(hongkai.dai): remove the warning when we change the solver backend.
  if (matrix_rows_ == 1) {
    drake::log()->warn(
        "LinearMatrixInequalityConstraint: the matrix has size 1. Please "
        "consider"
        "reformulating this as a linear inequality constraint for better "
        "speed/numerics.");
  } else if (matrix_rows_ == 2) {
    drake::log()->warn(
        "LinearMatrixInequalityConstraint: the matrix has size 2. Please "
        "consider reformulating this as a rotated Lorentz cone constraint for "
        "better speed/numerics.");
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
  std::tie(vars_, map_var_to_index_) =
      symbolic::ExtractVariablesFromExpression(expressions_);

  derivatives_ = symbolic::Jacobian(expressions_, vars_);

  // Setup the environment.
  for (int i = 0; i < vars_.size(); i++) {
    environment_.insert(vars_[i], 0.0);
  }
}

void ExpressionConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                  Eigen::VectorXd* y) const {
  DRAKE_THROW_UNLESS(x.rows() == vars_.rows());

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
  DRAKE_THROW_UNLESS(x.rows() == vars_.rows());

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
  DRAKE_THROW_UNLESS(x.rows() == vars_.rows());
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

std::string ExpressionConstraint::DoToLatex(
    const VectorX<symbolic::Variable>& vars, int precision) const {
  return ToLatexConstraint(*this, vars, precision);
}

ExponentialConeConstraint::ExponentialConeConstraint(
    const Eigen::Ref<const Eigen::SparseMatrix<double>>& A,
    const Eigen::Ref<const Eigen::Vector3d>& b)
    : Constraint(
          2, A.cols(), Eigen::Vector2d::Zero(),
          Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity())),
      A_{A},
      b_{b} {
  DRAKE_THROW_UNLESS(A.rows() == 3);
  set_is_thread_safe(true);
}

ExponentialConeConstraint::~ExponentialConeConstraint() = default;

template <typename DerivedX, typename ScalarY>
void ExponentialConeConstraint::DoEvalGeneric(
    const Eigen::MatrixBase<DerivedX>& x, VectorX<ScalarY>* y) const {
  y->resize(num_constraints());
  const Vector3<ScalarY> z = A_ * x.template cast<ScalarY>() + b_;
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

std::string ExponentialConeConstraint::DoToLatex(
    const VectorX<symbolic::Variable>& vars, int precision) const {
  const Vector3<symbolic::Expression> z = A_ * vars + b_;
  return fmt::format("0 \\le {},\\\\ {} \\le {}",
                     symbolic::ToLatex(z(1), precision),
                     symbolic::ToLatex(z(0), precision),
                     symbolic::ToLatex(z(1) * exp(z(2) / z(1)), precision));
}

}  // namespace solvers
}  // namespace drake
