#include "drake/systems/primitives/linear_system_internal.h"

namespace drake {
namespace systems {
namespace internal {

using Eigen::MatrixXd;
using Eigen::Ref;

// Note that unit testing of this code happens via its use in linear_system.h
// via the linear_system_test program.

MatrixXd ControllabilityMatrix(const Ref<const MatrixXd>& A,
                               const Ref<const MatrixXd>& B) {
  const int num_states = B.rows();
  const int num_inputs = B.cols();
  DRAKE_DEMAND(A.rows() == num_states);
  DRAKE_DEMAND(A.cols() == num_states);
  MatrixXd R(num_states, num_states * num_inputs);
  R.leftCols(num_inputs) = B;
  for (int i = 1; i < num_states; ++i) {
    R.middleCols(num_inputs * i, num_inputs) =
        A * R.middleCols(num_inputs * (i - 1), num_inputs);
  }
  return R;
}

bool IsControllable(const Ref<const MatrixXd>& A, const Ref<const MatrixXd>& B,
                    std::optional<double> threshold) {
  const MatrixXd R = ControllabilityMatrix(A, B);
  Eigen::ColPivHouseholderQR<MatrixXd> lu_decomp(R);
  if (threshold) {
    lu_decomp.setThreshold(threshold.value());
  }
  return lu_decomp.rank() == A.rows();
}

MatrixXd ObservabilityMatrix(const Ref<const MatrixXd>& A,
                             const Ref<const MatrixXd>& C) {
  const int num_states = C.cols();
  const int num_outputs = C.rows();
  DRAKE_DEMAND(A.rows() == num_states);
  DRAKE_DEMAND(A.cols() == num_states);
  MatrixXd O(num_states * num_outputs, num_states);
  O.topRows(num_outputs) = C;
  for (int i = 1; i < num_states; ++i) {
    O.middleRows(num_outputs * i, num_outputs) =
        O.middleRows(num_outputs * (i - 1), num_outputs) * A;
  }
  return O;
}

bool IsObservable(const Ref<const MatrixXd>& A, const Ref<const MatrixXd>& C,
                  std::optional<double> threshold) {
  const MatrixXd O = ObservabilityMatrix(A, C);
  Eigen::ColPivHouseholderQR<MatrixXd> lu_decomp(O);
  if (threshold) {
    lu_decomp.setThreshold(threshold.value());
  }
  return lu_decomp.rank() == A.rows();
}

bool IsStabilizable(const Ref<const MatrixXd>& A, const Ref<const MatrixXd>& B,
                    bool continuous_time, std::optional<double> threshold) {
  // (A, B) is stabilizable if [(λI - A) B] is full rank for all unstable
  // eigenvalues.
  DRAKE_DEMAND(A.rows() == A.cols());
  DRAKE_DEMAND(A.rows() == B.rows());
  Eigen::EigenSolver<MatrixXd> es(A, false);
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
          es.eigenvalues()(i) * MatrixXd::Identity(A.rows(), A.cols()) - A;
      mat.rightCols(B.cols()) = B;
      Eigen::ColPivHouseholderQR<Eigen::MatrixXcd> qr(mat);
      if (threshold) {
        qr.setThreshold(*threshold);
      }
      DRAKE_DEMAND(qr.info() == Eigen::Success);
      if (qr.rank() < A.rows()) {
        return false;
      }
    }
  }
  return true;
}

bool IsDetectable(const Ref<const MatrixXd>& A, const Ref<const MatrixXd>& C,
                  bool continuous_time, std::optional<double> threshold) {
  // The system (C, A) is detectable iff (A', C') is stabilizable.
  return IsStabilizable(A.transpose(), C.transpose(), continuous_time,
                        threshold);
}

}  // namespace internal
}  // namespace systems
}  // namespace drake
