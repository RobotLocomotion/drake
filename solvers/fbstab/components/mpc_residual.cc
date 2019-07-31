#include "drake/solvers/fbstab/components/mpc_residual.h"

#include <cmath>
#include <stdexcept>
#include <algorithm>

#include <Eigen/Dense>

#include "drake/solvers/fbstab/components/mpc_data.h"
#include "drake/solvers/fbstab/components/mpc_variable.h"

namespace drake {
namespace solvers {
namespace fbstab {

MPCResidual::MPCResidual(int N, int nx, int nu, int nc) {
  if (N <= 0 || nx <= 0 || nu <= 0 || nc <= 0) {
    throw std::runtime_error(
        "All inputs to MPCResidual::MPCResidual must be >= 1.");
  }
  N_ = N;
  nx_ = nx;
  nu_ = nu;
  nc_ = nc;
  nz_ = (N_ + 1) * (nx_ + nu_);
  nl_ = (N_ + 1) * nx_;
  nv_ = (N_ + 1) * nc_;

  z_.resize(nz_);
  l_.resize(nl_);
  v_.resize(nv_);

  z_.setConstant(0.0);
  l_.setConstant(0.0);
  v_.setConstant(0.0);
}

void MPCResidual::Fill(double a) {
  z_.setConstant(a);
  l_.setConstant(a);
  v_.setConstant(a);
}

void MPCResidual::Negate() {
  z_ *= -1;
  l_ *= -1;
  v_ *= -1;
}

double MPCResidual::Norm() const {
  return sqrt(znorm_ * znorm_ + lnorm_ * lnorm_ + vnorm_ * vnorm_);
}

double MPCResidual::Merit() const {
  const double temp = this->Norm();
  return 0.5 * temp * temp;
}

void MPCResidual::InnerResidual(const MPCVariable& x, const MPCVariable& xbar,
                                double sigma) {
  const MPCData* const data = x.data();
  if (xbar.data_ != data) {
    throw std::runtime_error(
        "In MPCResidual::InnerResidual: x and xbar have mismatched problem "
        "data.");
  }
  // r.z = H*z + f + G'*l + A'*v + sigma*(z-zbar)
  z_.setConstant(0.0);
  data->axpyf(1.0, &z_);
  data->gemvH(x.z(), 1.0, 1.0, &z_);
  data->gemvGT(x.l(), 1.0, 1.0, &z_);
  data->gemvAT(x.v(), 1.0, 1.0, &z_);
  z_.noalias() += sigma * (x.z() - xbar.z());

  // r.l = h - G*z + sigma(l - lbar)
  l_.setConstant(0.0);
  data->axpyh(1.0, &l_);
  data->gemvG(x.z(), -1.0, 1.0, &l_);
  l_.noalias() += sigma * (x.l() - xbar.l());

  // rv = phi(y + sigma*(v-vbar),v)
  for (int i = 0; i < nv_; i++) {
    const double ys = x.y()(i) + sigma * (x.v()(i) - xbar.v()(i));
    v_(i) = pfb(ys, x.v()(i), alpha_);
  }
  znorm_ = z_.norm();
  lnorm_ = l_.norm();
  vnorm_ = v_.norm();
}

void MPCResidual::NaturalResidual(const MPCVariable& x) {
  const MPCData* const data = x.data();
  // r.z = H*z + f + G'*l + A'*v
  z_.setConstant(0.0);
  data->axpyf(1.0, &z_);
  data->gemvH(x.z(), 1.0, 1.0, &z_);
  data->gemvGT(x.l(), 1.0, 1.0, &z_);
  data->gemvAT(x.v(), 1.0, 1.0, &z_);

  // r.l = h - G*z + sigma(l - lbar)
  l_.setConstant(0.0);
  data->axpyh(1.0, &l_);
  data->gemvG(x.z(), -1.0, 1.0, &l_);

  // rv = min(y,v)
  for (int i = 0; i < nv_; i++) {
    v_(i) = min(x.y()(i), x.v()(i)); // NOLINT
  }
  znorm_ = z_.norm();
  lnorm_ = l_.norm();
  vnorm_ = v_.norm();
}

void MPCResidual::PenalizedNaturalResidual(const MPCVariable& x) {
  const MPCData* const data = x.data();
  // r.z = H*z + f + G'*l + A'*v
  z_.setConstant(0.0);
  data->axpyf(1.0, &z_);
  data->gemvH(x.z(), 1.0, 1.0, &z_);
  data->gemvGT(x.l(), 1.0, 1.0, &z_);
  data->gemvAT(x.v(), 1.0, 1.0, &z_);

  // r.l = h - G*z + sigma(l - lbar)
  l_.setConstant(0.0);
  data->axpyh(1.0, &l_);
  data->gemvG(x.z(), -1.0, 1.0, &l_);

  // rv = alpha*min(y,v) + (1-alpha)*max(0,v)*max(0,y)
  for (int i = 0; i < nv_; i++) {
    const double nr = min(x.y()(i), x.v()(i));
    v_(i) = alpha_ * nr + (1 - alpha_) * max(0.0, x.y()(i)) * max(0, x.v()(i)); // NOLINT
  }
  znorm_ = z_.norm();
  lnorm_ = l_.norm();
  vnorm_ = v_.norm();
}

double MPCResidual::pfb(double a, double b, double alpha) {
  const double fb = a + b - sqrt(a * a + b * b);
  return alpha * fb + (1.0 - alpha) * max(0, a) * max(0, b);
}

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
