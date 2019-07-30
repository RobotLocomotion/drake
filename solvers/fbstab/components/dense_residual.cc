#include "drake/solvers/fbstab/components/dense_residual.h"

#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "drake/solvers/fbstab/components/dense_data.h"
#include "drake/solvers/fbstab/components/dense_variable.h"

namespace drake {
namespace solvers {
namespace fbstab {

DenseResidual::DenseResidual(int nz, int nv) {
  nz_ = nz;
  nv_ = nv;
  z_.resize(nz_);
  v_.resize(nv_);
}

void DenseResidual::Negate() {
  z_ *= -1.0;
  v_ *= -1.0;
}

void DenseResidual::NaturalResidual(const DenseVariable& x) {
  const DenseData* const data = x.data();
  // rz = H*z + f + A'*v
  // Calls are arranged to avoid creating temporaries.
  z_.noalias() = data->H() * x.z();
  z_.noalias() += data->f();
  z_.noalias() += data->A().transpose() * x.v();

  // rv = min(y,v)
  v_.noalias() = x.y().cwiseMin(x.v());

  znorm_ = z_.norm();
  vnorm_ = v_.norm();
}

void DenseResidual::PenalizedNaturalResidual(const DenseVariable& x) {
  NaturalResidual(x);
  for (int i = 0; i < nv_; i++) {
    v_(i) = alpha_ * v()(i) +
            (1.0 - alpha_) * max(0.0, x.y()(i)) * max(0.0, x.v()(i));
  }
  znorm_ = z_.norm();
  vnorm_ = v_.norm();
}

void DenseResidual::InnerResidual(const DenseVariable& x,
                                  const DenseVariable& xbar, double sigma) {
  const DenseData* const data = x.data();
  if (xbar.data() != data) {
    throw std::runtime_error(
        "In DenseResidual::InnerResidual: x and xbar have mismatched problem "
        "data.");
  }
  if (x.num_variables() != xbar.num_variables() ||
      x.num_constraints() != xbar.num_constraints()) {
    throw std::runtime_error("Size mismatch in DenseResidual::InnerResidual.");
  }
  if (sigma <= 0) {
    throw std::runtime_error(
        "In DenseResidual::InnerResidual: sigma must be positive.");
  }
  // rz = Hz + f + A'v + sigma(z - zbar)
  // Calls are arranged so as to avoid creating temporaries.
  z_.noalias() = data->H() * x.z();
  z_.noalias() += data->f();
  z_.noalias() += data->A().transpose() * x.v();
  z_.noalias() += sigma * (x.z() - xbar.z());

  // v_ = phi(ys,v), ys = y + sigma(x.v - xbar.v)
  for (int i = 0; i < nv_; i++) {
    const double ys = x.y()(i) + sigma * (x.v()(i) - xbar.v()(i));
    v_(i) = pfb(ys, x.v()(i), alpha_);
  }

  znorm_ = z_.norm();
  vnorm_ = v_.norm();
}

void DenseResidual::Fill(double a) {
  z_.setConstant(a);
  v_.setConstant(a);
}

double DenseResidual::Norm() const {
  return sqrt(znorm_ * znorm_ + vnorm_ * vnorm_);
}

double DenseResidual::Merit() const {
  const double temp = Norm();
  return 0.5 * temp * temp;
}

double DenseResidual::max(double a, double b) { return a > b ? a : b; }

double DenseResidual::min(double a, double b) { return a < b ? a : b; }

double DenseResidual::pfb(double a, double b, double alpha) {
  const double fb = a + b - sqrt(a * a + b * b);
  return alpha * fb + (1.0 - alpha) * max(0, a) * max(0, b);  // NOLINT
}

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
