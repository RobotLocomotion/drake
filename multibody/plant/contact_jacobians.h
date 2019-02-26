#pragma once

#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

template <class T>
struct ContactJacobians {
  /// Normal contact forces Jacobian.
  MatrixX<T> Jn;

  // Tangential contact forces Jacobian.
  MatrixX<T> Jt;

  // List of contact frames orientation R_WC in the world frame W.
  std::vector<Matrix3<T>> R_WC_list;
};

}  // namespace multibody
}  // namespace drake
