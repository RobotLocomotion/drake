#include "drake/solvers/fbstab/components/mpc_variable.h"

#include <cmath>
#include <memory>

#include <Eigen/Dense>

#include "drake/solvers/fbstab/components/mpc_data.h"

namespace drake {
namespace solvers {
namespace fbstab {

using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;

MPCVariable::MPCVariable(int N, int nx, int nu, int nc) {
  if (N <= 0 || nx <= 0 || nu <= 0 || nc <= 0) {
    throw std::runtime_error(
        "All size inputs to MPCVariable::MPCVariable must be >= 1.");
  }
  N_ = N;
  nx_ = nx;
  nu_ = nu;
  nc_ = nc;

  nz_ = (N_ + 1) * (nx_ + nu_);
  nl_ = (N_ + 1) * nx_;
  nv_ = (N_ + 1) * nc_;

  z_storage_ = std::make_unique<VectorXd>(nz_);
  l_storage_ = std::make_unique<VectorXd>(nl_);
  v_storage_ = std::make_unique<VectorXd>(nv_);
  y_storage_ = std::make_unique<VectorXd>(nv_);

  z_ = z_storage_.get();
  l_ = l_storage_.get();
  v_ = v_storage_.get();
  y_ = y_storage_.get();

  z_->setConstant(0.0);
  l_->setConstant(0.0);
  v_->setConstant(0.0);
  y_->setConstant(0.0);
}

MPCVariable::MPCVariable(VectorXd* z, VectorXd* l, VectorXd* v, VectorXd* y) {
  if (z == nullptr || l == nullptr || v == nullptr || y == nullptr) {
    throw std::runtime_error(
        "Inputs to MPCVariable::MPCVariable cannot be null.");
  }
  if (z->size() == 0 || l->size() == 0 || v->size() == 0 || y->size() == 0) {
    throw std::runtime_error(
        "All size inputs to MPCVariable::MPCVariable must be >= 1.");
  }
  if (v->size() != y->size()) {
    throw std::runtime_error(
        "In MPCVariable::MPCVariable, y and v must be the same size");
  }

  nz_ = z->size();
  nl_ = l->size();
  nv_ = v->size();

  z_ = z;
  l_ = l;
  v_ = v;
  y_ = y;
}

void MPCVariable::Fill(double a) {
  z_->setConstant(a);
  l_->setConstant(a);
  v_->setConstant(a);
  InitializeConstraintMargin();
}

void MPCVariable::InitializeConstraintMargin() {
  if (data_ == nullptr) {
    throw std::runtime_error(
        "Cannot call MPCVariable::InitializeConstraintMargin unless data is "
        "linked.");
  }
  // y = b - A*z
  y_->setConstant(0.0);
  data_->axpyb(1.0, y_);
  data_->gemvA(*z_, -1.0, 1.0, y_);
}

void MPCVariable::axpy(double a, const MPCVariable& x) {
  if (data_ == nullptr) {
    throw std::runtime_error(
        "Cannot call MPCVariable::axpy unless data is linked.");
  }

  z_->noalias() += a * (*x.z_);
  l_->noalias() += a * (*x.l_);
  v_->noalias() += a * (*x.v_);

  // y <- y + a*(x.y - b)
  y_->noalias() += a * (*x.y_);
  data_->axpyb(-a, y_);
}

void MPCVariable::Copy(const MPCVariable& x) {
  *z_ = *x.z_;
  *l_ = *x.l_;
  *v_ = *x.v_;
  *y_ = *x.y_;
  data_ = x.data_;
}

const MPCData* MPCVariable::data() const {
  if (data_ == nullptr) {
    throw std::runtime_error(
        "In MPCData::data: tried to access problem data before it's assigned.");
  }
  return data_;
}

void MPCVariable::ProjectDuals() { *v_ = v_->cwiseMax(0); }

double MPCVariable::Norm() const {
  const double t1 = z_->norm();
  const double t2 = l_->norm();
  const double t3 = v_->norm();

  return sqrt(t1 * t1 + t2 * t2 + t3 * t3);
}

bool MPCVariable::SameSize(const MPCVariable& x, const MPCVariable& y) {
  return (x.nz_ == y.nz_ && x.nl_ == y.nl_ && x.nv_ == y.nv_);
}

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
