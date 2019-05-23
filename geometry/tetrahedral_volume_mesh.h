#pragma once

#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
struct TetrahedralVolumeMesh {
  std::vector<Vector3<T>> vertices;
  std::vector<Vector4<int>> tetrahedra;
};

}  // namespace internal.
}  // namespace geometry.
}  // namespace drake.
