#include "drake/solvers/fbstab/components/dense_feasibility.h"

#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

#include "drake/solvers/fbstab/components/dense_data.h"
#include "drake/solvers/fbstab/components/dense_variable.h"

namespace drake {
namespace solvers {
namespace fbstab {

DenseFeasibility::DenseFeasibility(int nz, int nv) {
  if (nz <= 0 || nv <= 0) {
    throw std::runtime_error(
        "Inputs to DenseFeasibility::DenseFeasibility must be positive.");
  }
  nz_ = nz;
  nv_ = nv;
  z1_.resize(nz_);
  v1_.resize(nv_);
}

void DenseFeasibility::ComputeFeasibility(const DenseVariable& x, double tol) {
  if (tol <= 0) {
    throw std::runtime_error(
        "In DenseFeasibility::ComputeFeasibility: tol must be positive.");
  }
  const DenseData* const data = x.data();
  const Eigen::MatrixXd& H = data->H();
  const Eigen::MatrixXd& A = data->A();
  const Eigen::VectorXd& f = data->f();
  const Eigen::VectorXd& b = data->b();

  // The conditions for dual-infeasibility are:
  // max(Az) <= 0 and f'*z < 0 and ||Hz|| <= tol * ||z||
  v1_.noalias() = A * x.z();
  const double d1 = v1_.maxCoeff();

  const double d2 = f.dot(x.z());

  z1_.noalias() = H * x.z();
  const double d3 = z1_.lpNorm<Eigen::Infinity>();
  const double w = x.z().lpNorm<Eigen::Infinity>();

  if ((d1 <= tol * w) && (d2 < 0) && (d3 <= tol * w)) {
    dual_feasible_ = false;
  } else {
    dual_feasible_ = true;
  }

  // The conditions for primal infeasibility are:
  // v'*b < 0 and ||A'*v|| \leq tol * ||v||
  const double p1 = b.dot(x.v());

  z1_.noalias() = A.transpose() * x.v();
  const double p2 = z1_.lpNorm<Eigen::Infinity>();
  const double u = x.v().lpNorm<Eigen::Infinity>();

  if ((p1 < 0) && (p2 <= tol * u)) {
    primal_feasible_ = false;
  } else {
    primal_feasible_ = true;
  }
}

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
