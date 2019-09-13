#include "drake/solvers/fbstab/components/mpc_residual.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

#include "drake/solvers/fbstab/components/mpc_data.h"
#include "drake/solvers/fbstab/components/mpc_variable.h"

namespace drake {
namespace solvers {
namespace fbstab {

MpcResidual::MpcResidual(int N, int nx, int nu, int nc) {
  if (N <= 0 || nx <= 0 || nu <= 0 || nc <= 0) {
    throw std::runtime_error(
        "All inputs to MpcResidual::MpcResidual must be >= 1.");
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

  Fill(0.0);
}

void MpcResidual::Fill(double a) {
  z_.setConstant(a);
  l_.setConstant(a);
  v_.setConstant(a);
}

void MpcResidual::Negate() {
  z_ *= -1;
  l_ *= -1;
  v_ *= -1;
}

double MpcResidual::Norm() const {
  return sqrt(znorm_ * znorm_ + lnorm_ * lnorm_ + vnorm_ * vnorm_);
}

double MpcResidual::Merit() const {
  const double temp = this->Norm();
  return 0.5 * temp * temp;
}

void MpcResidual::InnerResidual(const MpcVariable& x, const MpcVariable& xbar,
                                double sigma) {
  const MpcData* const data = x.data();
  if (xbar.data_ != data) {
    throw std::runtime_error(
        "In MpcResidual::InnerResidual: x and xbar have mismatched problem "
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

void MpcResidual::NaturalResidual(const MpcVariable& x) {
  const MpcData* const data = x.data();
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
    v_(i) = std::min(x.y()(i), x.v()(i));
  }
  znorm_ = z_.norm();
  lnorm_ = l_.norm();
  vnorm_ = v_.norm();
}

void MpcResidual::PenalizedNaturalResidual(const MpcVariable& x) {
  NaturalResidual(x);

  for (int i = 0; i < nv_; i++) {
    v_(i) = alpha_ * v_(i) +
            (1 - alpha_) * std::max(0.0, x.y()(i)) * std::max(0.0, x.v()(i));
  }
  znorm_ = z_.norm();
  lnorm_ = l_.norm();
  vnorm_ = v_.norm();
}

double MpcResidual::pfb(double a, double b, double alpha) {
  double fb = a + b - sqrt(a * a + b * b);
  return alpha * fb + (1.0 - alpha) * std::max(0.0, a) * std::max(0.0, b);
}

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
