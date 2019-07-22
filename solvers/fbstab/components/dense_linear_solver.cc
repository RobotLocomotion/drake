#include "drake/solvers/fbstab/components/dense_linear_solver.h"

#include <cmath>

#include <Eigen/Dense>
#include "drake/solvers/fbstab/components/dense_data.h"
#include "drake/solvers/fbstab/components/dense_residual.h"
#include "drake/solvers/fbstab/components/dense_variable.h"

namespace drake {
namespace solvers {
namespace fbstab {

DenseLinearSolver::DenseLinearSolver(int nz, int nv) {
  if (nz <= 0 || nv <= 0) {
    throw std::runtime_error(
        "In DenseLinearSolver::DenseLinearSolver: inputs must be positive.");
  }
  nz_ = nz;
  nv_ = nv;

  K_.resize(nz_, nz_);
  r1_.resize(nz_);
  r2_.resize(nv_);
  Gamma_.resize(nv_);
  mus_.resize(nv_);
  gamma_.resize(nv_);
  B_.resize(nv_, nz_);
}

void DenseLinearSolver::SetAlpha(double alpha) { alpha_ = alpha; }

bool DenseLinearSolver::Factor(const DenseVariable& x,
                               const DenseVariable& xbar, double sigma) {
  const DenseData* const data = x.data();
  if (xbar.data() != data) {
    throw std::runtime_error(
        "In DenseLinearSolver::Factor: x and xbar have mismatched problem data.");
  }
  if (xbar.nz_ != x.nz_ || xbar.nv_ != x.nv_) {
    throw std::runtime_error(
        "In DenseLinearSolver::Factor: inputs must be the same size");
  }
  if (xbar.nz_ != nz_ || xbar.nv_ != nv_) {
    throw std::runtime_error(
        "In DenseLinearSolver::Factor: inputs must match object size.");
  }
  if (sigma <= 0) {
    throw std::runtime_error(
        "In DenseLinearSolver::Factor: sigma must be positive.");
  }
  const Eigen::MatrixXd& H = data->H();
  const Eigen::MatrixXd& A = data->A();

  K_ = H + sigma * Eigen::MatrixXd::Identity(nz_, nz_);

  // K <- K + A'*diag(Gamma(x))*A
  Point2D pfb_gradient;
  for (int i = 0; i < nv_; i++) {
    const double ys = x.y()(i) + sigma * (x.v()(i) - xbar.v()(i));
    pfb_gradient = PFBGradient(ys, x.v()(i));
    gamma_(i) = pfb_gradient.x;
    mus_(i) = pfb_gradient.y + sigma * pfb_gradient.x;
    Gamma_(i) = gamma_(i) / mus_(i);
  }
  // B is used to avoid temporaries
  B_.noalias() = Gamma_.asDiagonal() * A;
  K_.noalias() += A.transpose() * B_;

  // Factor K = LL' in place
  Eigen::LLT<Eigen::Ref<Eigen::MatrixXd> > L(K_);

  return true;
}

bool DenseLinearSolver::Solve(const DenseResidual& r, DenseVariable* x) const {
  if (x == nullptr) {
    throw std::runtime_error("In DenseLinearSolver::Solve: x cannot be null.");
  }
  if (r.nz_ != x->nz_ || r.nv_ != x->nv_) {
    throw std::runtime_error(
        "In DenseLinearSolver::Solve residual and variable objects must be the "
        "same size");
  }
  if (x->nz_ != nz_ || x->nv_ != nv_) {
    throw std::runtime_error(
        "In DenseLinearSolver::Factor: inputs must match object size.");
  }
  const DenseData* const data = x->data();
  const Eigen::MatrixXd& A = data->A();
  const Eigen::VectorXd& b = data->b();

  // This method solves the system:
  // KK'z = rz - A'*diag(1/mus)*rv
  // diag(mus) v = rv + diag(gamma)*A*z
  // Where K has been precomputed by the factor routine.
  // See (28) and (29) in https://arxiv.org/pdf/1901.04046.pdf

  // Compute rz - A'*(rv./mus) and store it in r1_.
  r2_ = r.v_.cwiseQuotient(mus_);
  r1_.noalias() = r.z_ - A.transpose() * r2_;

  // Solve KK'*z = rz - A'*(rv./mus)
  // where K = chol(H + sigma*I + A'*Gamma*A)
  // is assumed to have been computed during the factor phase.
  x->z() = r1_;
  CholeskySolve(K_, x->z_);

  // Compute v = diag(1/mus) * (rv + diag(gamma)*A*z)
  // written so as to avoid temporary creation
  r2_.noalias() = A * x->z();
  r2_.noalias() = gamma_.asDiagonal() * r2_;
  r2_.noalias() += r.v_;

  // v = r2./mus
  x->v() = r2_.cwiseQuotient(mus_);

  // y = b - Az
  x->y() = b - A * x->z();

  return true;
}

void DenseLinearSolver::CholeskySolve(const Eigen::MatrixXd& A,
                                      Eigen::VectorXd* b) const {
  A.triangularView<Eigen::Lower>().solveInPlace(*b);
  A.triangularView<Eigen::Lower>().transpose().solveInPlace(*b);
}

DenseLinearSolver::Point2D DenseLinearSolver::PFBGradient(double a, double b) {
  double y = 0;
  double x = 0;
  const double r = sqrt(a * a + b * b);
  const double d = 1.0 / sqrt(2.0);

  if (r < zero_tolerance_) {
    x = alpha_ * (1.0 - d);
    y = alpha_ * (1.0 - d);

  } else if ((a > 0) && (b > 0)) {
    x = alpha_ * (1.0 - a / r) + (1.0 - alpha_) * b;
    y = alpha_ * (1.0 - b / r) + (1.0 - alpha_) * a;

  } else {
    x = alpha_ * (1.0 - a / r);
    y = alpha_ * (1.0 - b / r);
  }

  Point2D output = {x, y};
  return output;
}

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
