#include "drake/solvers/fbstab/components/mpc_feasibility.h"

#include <algorithm>
#include <stdexcept>

#include <Eigen/Dense>

#include "drake/solvers/fbstab/components/mpc_data.h"
#include "drake/solvers/fbstab/components/mpc_variable.h"

namespace drake {
namespace solvers {
namespace fbstab {

MpcFeasibility::MpcFeasibility(int N, int nx, int nu, int nc) {
  if (N <= 0 || nx <= 0 || nu <= 0 || nc <= 0) {
    throw std::runtime_error(
        "All size inputs to MpcFeasibility::MpcFeasibility must be >= 1.");
  }
  nx_ = nx;
  nu_ = nu;
  nc_ = nc;
  N_ = N;

  nz_ = (nx_ + nu_) * (N_ + 1);
  nl_ = nx_ * (N_ + 1);
  nv_ = nc_ * (N_ + 1);

  tz_.resize(nz_);
  tl_.resize(nl_);
  tv_.resize(nv_);
}

void MpcFeasibility::ComputeFeasibility(const MpcVariable& x, double tol) {
  const MpcData* const data = x.data();
  if (x.N_ != N_ || x.nx_ != nx_ || x.nu_ != nu_ || x.nc_ != nc_) {
    throw std::runtime_error(
        "In MpcFeasibility::ComputeFeasibility: size mismatch between *this "
        "and x.");
  }
  // The conditions for dual-infeasibility are:
  // max(Az) <= 0 and f'*z < 0 and |Hz| <= tol * |z| and |Gz| <= tol*|z|

  // Compute d1 = max(Az).
  data->gemvA(x.z(), 1.0, 0.0, &tv_);
  const double d1 = tv_.maxCoeff();

  // Compute d2 = infnorm(Gz).
  data->gemvG(x.z(), 1.0, 0.0, &tl_);
  const double d2 = tl_.lpNorm<Eigen::Infinity>();

  // Compute d3 = infnorm(Hz).
  data->gemvH(x.z(), 1.0, 0.0, &tz_);
  const double d3 = tz_.lpNorm<Eigen::Infinity>();

  // Compute d4 = f'*z
  tz_.setConstant(0.0);
  data->axpyf(1.0, &tz_);
  const double d4 = tz_.dot(x.z());

  double w = x.z().lpNorm<Eigen::Infinity>();
  if ((d1 <= w * tol) && (d2 <= tol * w) && (d3 <= tol * w) && (d4 < 0) &&
      (w > 1e-14)) {
    dual_feasible_ = false;
  } else {
    dual_feasible_ = true;
  }

  // The conditions for primal infeasibility are:
  // v'*b + l'*h < 0 and |A'*v + G'*l| \leq tol * |(v,l)|

  // Compute p1 = infnorm(G'*l + A'*v).
  tz_.fill(0.0);
  data->gemvAT(x.v(), 1.0, 1.0, &tz_);
  data->gemvGT(x.l(), 1.0, 1.0, &tz_);
  const double p1 = tz_.lpNorm<Eigen::Infinity>();

  // Compute p2 = v'*b + l'*h.
  tv_.setConstant(0.0);
  data->axpyb(1.0, &tv_);
  tl_.setConstant(0.0);
  data->axpyh(1.0, &tl_);
  const double p2 = tl_.dot(x.l()) + tv_.dot(x.v());

  const double u = std::max(x.v().lpNorm<Eigen::Infinity>(),
                            x.l().lpNorm<Eigen::Infinity>());
  if ((p1 <= tol * u) && (p2 < 0)) {
    primal_feasible_ = false;
  } else {
    primal_feasible_ = true;
  }
}

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
