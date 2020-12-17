#include "drake/solvers/cost.h"

#include <memory>

#include "drake/math/autodiff_gradient.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::make_shared;
using std::shared_ptr;

namespace drake {
namespace solvers {
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
}  // namespace

std::ostream& LinearCost::DoDisplay(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  return DisplayCost(*this, os, "LinearCost", vars);
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
  const MatrixXd dx = math::autoDiffToGradientMatrix(x);
  const Eigen::VectorXd x_val = drake::math::autoDiffToValueMatrix(x);
  const Eigen::RowVectorXd xT_times_Q = x_val.transpose() * Q_;
  const Vector1d result(.5 * xT_times_Q.dot(x_val) + b_.dot(x_val) + c_);
  const Eigen::RowVectorXd dy = xT_times_Q + b_.transpose();

  // If dx is the identity matrix (very common here), then skip the chain rule
  // multiplication dy * dx
  if (dx.rows() == x.size() && dx.cols() == x.size() &&
      dx == MatrixXd::Identity(x.size(), x.size())) {
    *y = math::initializeAutoDiffGivenGradientMatrix(result, dy);
  } else {
    *y = math::initializeAutoDiffGivenGradientMatrix(result, dy * dx);
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

shared_ptr<QuadraticCost> MakeQuadraticErrorCost(
    const Eigen::Ref<const MatrixXd>& Q,
    const Eigen::Ref<const VectorXd>& x_desired) {
  const double c = x_desired.dot(Q * x_desired);
  return make_shared<QuadraticCost>(2 * Q, -2 * Q * x_desired, c);
}

shared_ptr<QuadraticCost> MakeL2NormCost(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b) {
  const double c = b.dot(b);
  return make_shared<QuadraticCost>(2 * A.transpose() * A,
                                    -2 * A.transpose() * b, c);
}

}  // namespace solvers
}  // namespace drake
