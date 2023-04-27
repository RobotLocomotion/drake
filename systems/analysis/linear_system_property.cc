#include "drake/systems/analysis/linear_system_property.h"

#include <complex>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

bool Controllable(const Eigen::Ref<const Eigen::MatrixXd>& A,
                  const Eigen::Ref<const Eigen::MatrixXd>& B) {
  DRAKE_DEMAND(A.rows() == A.cols());
  DRAKE_DEMAND(A.rows() == B.rows());
  // controllability_matrix = [B AB A²B ... Aⁿ⁻¹B]
  Eigen::MatrixXd controllability_matrix(A.rows(), A.rows() * B.cols());
  controllability_matrix.leftCols(B.cols()) = B;
  for (int i = 1; i < A.rows(); ++i) {
    controllability_matrix.middleCols(B.cols() * i, B.cols()) =
        A * controllability_matrix.middleCols(B.cols() * (i - 1), B.cols());
  }
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(controllability_matrix);
  DRAKE_DEMAND(qr.info() == Eigen::Success);
  return qr.rank() == A.rows();
}

bool Observable(const Eigen::Ref<const Eigen::MatrixXd>& A,
                const Eigen::Ref<const Eigen::MatrixXd>& C) {
  // (A, C) is observable if (A', C') is controllable.
  return Controllable(A.transpose(), C.transpose());
}

bool Stabilizable(const Eigen::Ref<const Eigen::MatrixXd>& A,
                  const Eigen::Ref<const Eigen::MatrixXd>& B,
                  bool continuous_time) {
  // (A, B) is stabilizable if [(λI - A) B] is full rank for all unstable
  // eigenvalues.
  DRAKE_DEMAND(A.rows() == A.cols());
  DRAKE_DEMAND(A.rows() == B.rows());
  Eigen::EigenSolver<Eigen::MatrixXd> es(A);
  DRAKE_DEMAND(es.info() == Eigen::Success);
  for (int i = 0; i < es.eigenvalues().size(); ++i) {
    bool stable_mode = false;
    if (continuous_time) {
      stable_mode = es.eigenvalues()(i).real() < 0;
    } else {
      stable_mode = std::norm(es.eigenvalues()(i)) < 1;
    }
    if (!stable_mode) {
      Eigen::MatrixXcd mat(A.rows(), A.cols() + B.cols());
      mat.leftCols(A.cols()) =
          es.eigenvalues()(i) * Eigen::MatrixXd::Identity(A.rows(), A.cols()) -
          A;
      mat.rightCols(B.cols()) = B;
      Eigen::ColPivHouseholderQR<Eigen::MatrixXcd> qr(mat);
      DRAKE_DEMAND(qr.info() == Eigen::Success);
      if (qr.rank() < A.rows()) {
        return false;
      }
    }
  }
  return true;
}

bool Detectable(const Eigen::Ref<const Eigen::MatrixXd>& A,
                const Eigen::Ref<const Eigen::MatrixXd>& C,
                bool continuous_time) {
  // (A, C) is detectable if (A', C') is stabilizable.
  return Stabilizable(A.transpose(), C.transpose(), continuous_time);
}
}  // namespace systems
}  // namespace drake
