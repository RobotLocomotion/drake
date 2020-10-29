#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace pydrake {
namespace internal {

template <typename T>
VectorX<T> Solve(const MatrixX<T>& A, const VectorX<T>& b) {
  // This solver seems the closest to the latest NumPy's implementation
  // of np.linalg.solve(). For more info, see this comment:
  // https://github.com/RobotLocomotion/drake/issues/14264#issuecomment-719036358
  DRAKE_THROW_UNLESS(A.rows() == A.cols());
  DRAKE_THROW_UNLESS(b.rows() == A.rows());
  return Eigen::PartialPivLU<MatrixX<T>>(A).solve(b);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
