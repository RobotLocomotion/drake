#include "drake/solvers/fbstab/components/dense_data.h"

#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

namespace drake {
namespace solvers {
namespace fbstab {

using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;

DenseData::DenseData(const MatrixXd* H, const VectorXd* f, const MatrixXd* A,
                     const VectorXd* b)
    : H_{H}, f_{f}, A_{A}, b_{b} {
  if (H == nullptr || f == nullptr || A == nullptr || b == nullptr) {
    throw std::runtime_error("Inputs to DenseData::DenseData cannot be null.");
  }
  if (H->rows() != H->cols() || H->rows() != f->size()) {
    throw std::runtime_error(
        "In DenseData::DenseData: H must be square and the same size as f.");
  }
  if (A->cols() != H->rows() || A->rows() != b->size()) {
    throw std::runtime_error(
        "In DenseData::DenseData: Sizing of data defining Az <= b is "
        "inconsistent.");
  }

  nz_ = f->size();
  nv_ = b->size();
}

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
