#include "drake/solvers/fbstab/components/dense_variable.h"

#include <cmath>
#include <memory>
#include <stdexcept>

#include <Eigen/Dense>

#include "drake/solvers/fbstab/components/dense_data.h"

namespace drake {
namespace solvers {
namespace fbstab {

using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;

DenseVariable::DenseVariable(int nz, int nv) {
  if (nz <= 0 || nv <= 0) {
    throw std::runtime_error(
        "Inputs nz and nv to DenseVariable::DenseVariable must be positive.");
  }
  nz_ = nz;
  nv_ = nv;

  z_storage_ = std::make_unique<VectorXd>(nz_);
  v_storage_ = std::make_unique<VectorXd>(nv_);
  y_storage_ = std::make_unique<VectorXd>(nv_);

  z_ = z_storage_.get();
  v_ = v_storage_.get();
  y_ = y_storage_.get();
}

DenseVariable::DenseVariable(VectorXd* z, VectorXd* v, VectorXd* y) {
  if (z == nullptr || v == nullptr || y == nullptr) {
    throw std::runtime_error(
        "DenseVariable::DenseVariable requires non-null pointers.");
  }
  if (y->size() != v->size()) {
    throw std::runtime_error(
        "In DenseVariable::DenseVariable: v and y input size mismatch");
  }
  nz_ = z->size();
  nv_ = v->size();

  if (nz_ == 0 || nv_ == 0) {
    throw std::runtime_error(
        "Inputs to DenseVariable::DenseVariable must have nonzero sizes.");
  }
  z_ = z;
  v_ = v;
  y_ = y;
}

void DenseVariable::Fill(double a) {
  if (data_ == nullptr) {
    throw std::runtime_error(
        "Cannot call DenseVariable::Fill unless data is linked.");
  }
  z_->setConstant(a);
  v_->setConstant(a);

  // Compute y = b - A*z
  if (a == 0.0) {
    *y_ = data_->b();
  } else {
    InitializeConstraintMargin();
  }
}

void DenseVariable::InitializeConstraintMargin() {
  if (data_ == nullptr) {
    throw std::runtime_error(
        "Cannot call DenseVariable::InitializeConstraintMargin unless data is "
        "linked.");
  }
  y_->noalias() = data_->b() - data_->A() * (*z_);
}

void DenseVariable::axpy(double a, const DenseVariable& x) {
  if (data_ == nullptr) {
    throw std::runtime_error(
        "Cannot call DenseVariable::axpy unless data is linked.");
  }
  (*z_) += a * x.z();
  (*v_) += a * x.v();
  (*y_) += a * (x.y() - data_->b());
}

void DenseVariable::Copy(const DenseVariable& x) {
  if (nz_ != x.nz_ || nv_ != x.nv_) {
    throw std::runtime_error("Sizes not equal in DenseVariable::Copy");
  }
  (*z_) = x.z();
  (*v_) = x.v();
  (*y_) = x.y();
  data_ = x.data_;
}

const DenseData* DenseVariable::data() const {
  if (data_ == nullptr) {
    throw std::runtime_error(
        "In DenseData::data: pointer to data requested before being assigned.");
  }

  return data_;
}
void DenseVariable::ProjectDuals() { *v_ = v_->cwiseMax(0); }

double DenseVariable::Norm() const {
  double t1 = z_->norm();
  double t2 = v_->norm();
  return sqrt(t1 * t1 + t2 * t2);
}

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
