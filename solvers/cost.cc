#include "drake/solvers/cost.h"

#include <memory>

#include "drake/common/symbolic/latex.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/differentiable_norm.h"
#include "drake/solvers/constraint.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::make_shared;
using std::shared_ptr;

namespace drake {
namespace solvers {

namespace {
std::ostream& DisplayCost(const Cost& cost, std::ostream& os,
                          const std::string& name,
                          const VectorX<symbolic::Variable>& vars) {
  os << name;
  // Append the expression.
  VectorX<symbolic::Expression> e;
  cost.Eval(vars, &e);
  DRAKE_DEMAND(e.size() == 1);
  os << " " << e[0];

  // Append the description (when provided).
  const std::string& description = cost.get_description();
  if (!description.empty()) {
    os << " described as '" << description << "'";
  }

  return os;
}

std::string ToLatexCost(const Cost& cost,
                        const VectorX<symbolic::Variable>& vars,
                        int precision) {
  VectorX<symbolic::Expression> e;
  cost.Eval(vars, &e);
  DRAKE_DEMAND(e.size() == 1);
  std::string latex = symbolic::ToLatex(e[0], precision);
  return latex;
}

}  // namespace

LinearCost::~LinearCost() = default;

void LinearCost::UpdateCoefficients(
    const Eigen::Ref<const Eigen::VectorXd>& new_a, double new_b) {
  if (new_a.rows() != a_.rows()) {
    throw std::runtime_error("Can't change the number of decision variables");
  }

  a_ = new_a;
  b_ = new_b;
}

void LinearCost::update_coefficient_entry(int i, double val) {
  DRAKE_DEMAND(i >= 0 && i < a_.rows());
  a_[i] = val;
}

void LinearCost::update_constant_term(double new_b) {
  b_ = new_b;
}

template <typename DerivedX, typename U>
void LinearCost::DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                               VectorX<U>* y) const {
  y->resize(1);
  (*y)(0) = a_.dot(x) + b_;
}

void LinearCost::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                        Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}
void LinearCost::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                        AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}

void LinearCost::DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
                        VectorX<symbolic::Expression>* y) const {
  DoEvalGeneric(x, y);
}

std::ostream& LinearCost::DoDisplay(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  return DisplayCost(*this, os, "LinearCost", vars);
}

std::string LinearCost::DoToLatex(const VectorX<symbolic::Variable>& vars,
                                  int precision) const {
  return ToLatexCost(*this, vars, precision);
}

QuadraticCost::~QuadraticCost() = default;

void QuadraticCost::UpdateHessianEntry(int i, int j, double val,
                                       std::optional<bool> is_hessian_psd) {
  DRAKE_DEMAND(i >= 0 && i < Q_.rows());
  DRAKE_DEMAND(j >= 0 && j < Q_.rows());
  Q_(i, j) = val;
  Q_(j, i) = val;
  if (is_hessian_psd.has_value()) {
    is_convex_ = is_hessian_psd.value();
  } else {
    is_convex_ = CheckHessianPsd();
  }
}

void QuadraticCost::update_linear_coefficient_entry(int i, double val) {
  DRAKE_DEMAND(i >= 0 && i < b_.rows());
  b_(i) = val;
}

void QuadraticCost::update_constant_term(double new_c) {
  c_ = new_c;
}

template <typename DerivedX, typename U>
void QuadraticCost::DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                                  VectorX<U>* y) const {
  y->resize(1);
  *y = .5 * x.transpose() * Q_ * x + b_.transpose() * x;
  (*y)(0) += c_;
}

void QuadraticCost::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                           Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void QuadraticCost::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                           AutoDiffVecXd* y) const {
  // Specialized evaluation of cost and gradient
  const MatrixXd dx = math::ExtractGradient(x);
  const Eigen::VectorXd x_val = drake::math::ExtractValue(x);
  const Eigen::RowVectorXd xT_times_Q = x_val.transpose() * Q_;
  const Vector1d result(.5 * xT_times_Q.dot(x_val) + b_.dot(x_val) + c_);
  const Eigen::RowVectorXd dy = xT_times_Q + b_.transpose();

  // If dx is the identity matrix (very common here), then skip the chain rule
  // multiplication dy * dx
  if (dx.rows() == x.size() && dx.cols() == x.size() &&
      dx == MatrixXd::Identity(x.size(), x.size())) {
    *y = math::InitializeAutoDiff(result, dy);
  } else {
    *y = math::InitializeAutoDiff(result, dy * dx);
  }
}

void QuadraticCost::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  DoEvalGeneric(x, y);
}

std::ostream& QuadraticCost::DoDisplay(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  return DisplayCost(*this, os, "QuadraticCost", vars);
}

std::string QuadraticCost::DoToLatex(const VectorX<symbolic::Variable>& vars,
                                     int precision) const {
  return ToLatexCost(*this, vars, precision);
}

bool QuadraticCost::CheckHessianPsd() {
  Eigen::LDLT<Eigen::MatrixXd> ldlt_solver;
  ldlt_solver.compute(Q_);
  return ldlt_solver.isPositive();
}

shared_ptr<QuadraticCost> MakeQuadraticErrorCost(
    const Eigen::Ref<const MatrixXd>& Q,
    const Eigen::Ref<const VectorXd>& x_desired) {
  const double c = x_desired.dot(Q * x_desired);
  return make_shared<QuadraticCost>(2 * Q, -2 * Q * x_desired, c);
}

shared_ptr<QuadraticCost> Make2NormSquaredCost(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b) {
  const double c = b.dot(b);
  return make_shared<QuadraticCost>(2 * A.transpose() * A,
                                    -2 * A.transpose() * b, c,
                                    true /* Hessian is psd */);
}

L1NormCost::L1NormCost(const Eigen::Ref<const Eigen::MatrixXd>& A,
                       const Eigen::Ref<const Eigen::VectorXd>& b)
    : Cost(A.cols()), A_(A), b_(b) {
  set_is_thread_safe(true);
  DRAKE_THROW_UNLESS(A_.rows() == b_.rows());
}

L1NormCost::~L1NormCost() = default;

void L1NormCost::UpdateCoefficients(
    const Eigen::Ref<const Eigen::MatrixXd>& new_A,
    const Eigen::Ref<const Eigen::VectorXd>& new_b) {
  if (new_A.cols() != A_.cols()) {
    throw std::runtime_error("Can't change the number of decision variables");
  }
  if (new_A.rows() != new_b.rows()) {
    throw std::runtime_error("A and b must have the same number of rows.");
  }

  A_ = new_A;
  b_ = new_b;
}

void L1NormCost::update_A_entry(int i, int j, double val) {
  DRAKE_DEMAND(i >= 0 && i < A_.rows());
  DRAKE_DEMAND(j >= 0 && j < A_.cols());
  A_(i, j) = val;
}

void L1NormCost::update_b_entry(int i, double val) {
  DRAKE_DEMAND(i >= 0 && i < b_.rows());
  b_(i) = val;
}

void L1NormCost::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                        Eigen::VectorXd* y) const {
  y->resize(1);
  (*y)(0) = (A_ * x + b_).lpNorm<1>();
}

void L1NormCost::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                        AutoDiffVecXd* y) const {
  y->resize(1);
  (*y)(0) = (A_ * x + b_).lpNorm<1>();
}

void L1NormCost::DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
                        VectorX<symbolic::Expression>* y) const {
  y->resize(1);
  (*y)(0) = (A_ * x + b_).cwiseAbs().sum();
}

std::ostream& L1NormCost::DoDisplay(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  return DisplayCost(*this, os, "L1NormCost", vars);
}

std::string L1NormCost::DoToLatex(const VectorX<symbolic::Variable>& vars,
                                  int precision) const {
  return fmt::format("\\left|{}\\right|_1",
                     symbolic::ToLatex((A_ * vars + b_).eval(), precision));
}

L2NormCost::L2NormCost(const Eigen::Ref<const Eigen::MatrixXd>& A,
                       const Eigen::Ref<const Eigen::VectorXd>& b)
    : Cost(A.cols()), A_(A), b_(b) {
  set_is_thread_safe(true);
  DRAKE_THROW_UNLESS(A_.get_as_sparse().rows() == b_.rows());
}

L2NormCost::L2NormCost(const Eigen::SparseMatrix<double>& A,
                       const Eigen::Ref<const Eigen::VectorXd>& b)
    : Cost(A.cols()), A_(A), b_(b) {
  set_is_thread_safe(true);
  DRAKE_THROW_UNLESS(A_.get_as_sparse().rows() == b_.rows());
}

L2NormCost::~L2NormCost() = default;

void L2NormCost::UpdateCoefficients(
    const Eigen::Ref<const Eigen::MatrixXd>& new_A,
    const Eigen::Ref<const Eigen::VectorXd>& new_b) {
  if (new_A.cols() != A_.get_as_sparse().cols()) {
    throw std::runtime_error("Can't change the number of decision variables");
  }
  if (new_A.rows() != new_b.rows()) {
    throw std::runtime_error("A and b must have the same number of rows.");
  }
  A_ = new_A;
  b_ = new_b;
}

void L2NormCost::UpdateCoefficients(
    const Eigen::SparseMatrix<double>& new_A,
    const Eigen::Ref<const Eigen::VectorXd>& new_b) {
  if (new_A.cols() != A_.get_as_sparse().cols()) {
    throw std::runtime_error("Can't change the number of decision variables");
  }
  if (new_A.rows() != new_b.rows()) {
    throw std::runtime_error("A and b must have the same number of rows.");
  }
  A_ = new_A;
  b_ = new_b;
}

void L2NormCost::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                        Eigen::VectorXd* y) const {
  y->resize(1);
  (*y)(0) = (A_.get_as_sparse() * x + b_).norm();
}

void L2NormCost::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                        AutoDiffVecXd* y) const {
  y->resize(1);
  (*y)(0) = math::DifferentiableNorm(A_.get_as_sparse() * x + b_);
}

void L2NormCost::DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
                        VectorX<symbolic::Expression>* y) const {
  y->resize(1);
  (*y)(0) = sqrt((A_.get_as_sparse() * x + b_).squaredNorm());
}

std::ostream& L2NormCost::DoDisplay(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  return DisplayCost(*this, os, "L2NormCost", vars);
}

std::string L2NormCost::DoToLatex(const VectorX<symbolic::Variable>& vars,
                                  int precision) const {
  return fmt::format(
      "\\left|{}\\right|_2",
      symbolic::ToLatex((A_.get_as_sparse() * vars + b_).eval(), precision));
}

LInfNormCost::LInfNormCost(const Eigen::Ref<const Eigen::MatrixXd>& A,
                           const Eigen::Ref<const Eigen::VectorXd>& b)
    : Cost(A.cols()), A_(A), b_(b) {
  set_is_thread_safe(true);
  DRAKE_THROW_UNLESS(A_.rows() == b_.rows());
}

LInfNormCost::~LInfNormCost() = default;

void LInfNormCost::UpdateCoefficients(
    const Eigen::Ref<const Eigen::MatrixXd>& new_A,
    const Eigen::Ref<const Eigen::VectorXd>& new_b) {
  if (new_A.cols() != A_.cols()) {
    throw std::runtime_error("Can't change the number of decision variables");
  }
  if (new_A.rows() != new_b.rows()) {
    throw std::runtime_error("A and b must have the same number of rows.");
  }

  A_ = new_A;
  b_ = new_b;
}

void LInfNormCost::update_A_entry(int i, int j, double val) {
  DRAKE_DEMAND(i >= 0 && i < A_.rows());
  DRAKE_DEMAND(j >= 0 && j < A_.cols());
  A_(i, j) = val;
}

void LInfNormCost::update_b_entry(int i, double val) {
  DRAKE_DEMAND(i >= 0 && i < b_.rows());
  b_(i) = val;
}

void LInfNormCost::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                          Eigen::VectorXd* y) const {
  y->resize(1);
  (*y)(0) = (A_ * x + b_).lpNorm<Eigen::Infinity>();
}

void LInfNormCost::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                          AutoDiffVecXd* y) const {
  y->resize(1);
  (*y)(0) = (A_ * x + b_).lpNorm<Eigen::Infinity>();
}

void LInfNormCost::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  y->resize(1);
  (*y)(0) = (A_ * x + b_).cwiseAbs().maxCoeff();
}

std::ostream& LInfNormCost::DoDisplay(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  return DisplayCost(*this, os, "LInfNormCost", vars);
}

std::string LInfNormCost::DoToLatex(const VectorX<symbolic::Variable>& vars,
                                    int precision) const {
  return fmt::format("\\left|{}\\right|_\\infty",
                     symbolic::ToLatex((A_ * vars + b_).eval(), precision));
}

PerspectiveQuadraticCost::PerspectiveQuadraticCost(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b)
    : Cost(A.cols()), A_(A), b_(b) {
  set_is_thread_safe(true);
  DRAKE_THROW_UNLESS(A_.rows() >= 2);
  DRAKE_THROW_UNLESS(A_.rows() == b_.rows());
}

PerspectiveQuadraticCost::~PerspectiveQuadraticCost() = default;

void PerspectiveQuadraticCost::UpdateCoefficients(
    const Eigen::Ref<const Eigen::MatrixXd>& new_A,
    const Eigen::Ref<const Eigen::VectorXd>& new_b) {
  if (new_A.cols() != A_.cols()) {
    throw std::runtime_error("Can't change the number of decision variables");
  }
  if (new_A.rows() != new_b.rows()) {
    throw std::runtime_error("A and b must have the same number of rows.");
  }

  A_ = new_A;
  b_ = new_b;
}

void PerspectiveQuadraticCost::update_A_entry(int i, int j, double val) {
  DRAKE_DEMAND(i >= 0 && i < A_.rows());
  DRAKE_DEMAND(j >= 0 && j < A_.cols());
  A_(i, j) = val;
}

void PerspectiveQuadraticCost::update_b_entry(int i, double val) {
  DRAKE_DEMAND(i >= 0 && i < b_.rows());
  b_(i) = val;
}

template <typename DerivedX, typename U>
void PerspectiveQuadraticCost::DoEvalGeneric(
    const Eigen::MatrixBase<DerivedX>& x, VectorX<U>* y) const {
  y->resize(1);
  VectorX<U> z = A_ * x.template cast<U>() + b_;
  (*y)(0) = z.tail(z.size() - 1).squaredNorm() / z(0);
}

void PerspectiveQuadraticCost::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void PerspectiveQuadraticCost::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                      AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}

void PerspectiveQuadraticCost::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  DoEvalGeneric(x, y);
}

std::ostream& PerspectiveQuadraticCost::DoDisplay(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  return DisplayCost(*this, os, "PerspectiveQuadraticCost", vars);
}

std::string PerspectiveQuadraticCost::DoToLatex(
    const VectorX<symbolic::Variable>& vars, int precision) const {
  return ToLatexCost(*this, vars, precision);
}

ExpressionCost::ExpressionCost(const symbolic::Expression& e)
    : Cost(e.GetVariables().size()),
      /* We reuse the Constraint evaluator's implementation. */
      evaluator_(std::make_unique<ExpressionConstraint>(
          Vector1<symbolic::Expression>{e},
          /* The ub, lb are unused but still required. */
          Vector1d(0.0), Vector1d(0.0))) {
  set_is_thread_safe(evaluator_->is_thread_safe());
}

const VectorXDecisionVariable& ExpressionCost::vars() const {
  return dynamic_cast<const ExpressionConstraint&>(*evaluator_).vars();
}

const symbolic::Expression& ExpressionCost::expression() const {
  return dynamic_cast<const ExpressionConstraint&>(*evaluator_)
      .expressions()
      .coeffRef(0);
}

void ExpressionCost::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                            Eigen::VectorXd* y) const {
  evaluator_->Eval(x, y);
}

void ExpressionCost::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                            AutoDiffVecXd* y) const {
  evaluator_->Eval(x, y);
}

void ExpressionCost::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  evaluator_->Eval(x, y);
}

std::ostream& ExpressionCost::DoDisplay(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  return DisplayCost(*this, os, "ExpressionCost", vars);
}

std::string ExpressionCost::DoToLatex(const VectorX<symbolic::Variable>& vars,
                                      int precision) const {
  return ToLatexCost(*this, vars, precision);
}

}  // namespace solvers
}  // namespace drake
