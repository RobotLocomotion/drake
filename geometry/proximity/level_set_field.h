#pragma once

#include <functional>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {

// TODO(amcastro-tri): consider making this an abstract class so that we can
// inherit multiple implementations (analytical, structured grids, etc.)
template <typename T>
class LevelSetField {
 public:
  LevelSetField(std::function<T(const Vector3<T>&)> level_set_M,
                std::function<Vector3<T>(const Vector3<T>&)> grad_level_set_M)
      : level_set_M_(level_set_M), grad_level_set_M_(grad_level_set_M) {}

  LevelSetField(LevelSetField&&) = default;
  LevelSetField& operator=(LevelSetField&&) = default;

  const std::function<T(const Vector3<T>&)>& level_set_M() const {
    return level_set_M_;
  }

  const std::function<Vector3<T>(const Vector3<T>&)>& grad_level_set_M() const {
    return grad_level_set_M_;
  }

  T CalcLevelSet(const Vector3<T>& p_MQ) const {
    return level_set_M_(p_MQ);
  }

  /// Computes the gradient of the level set function expressed in the frame M
  /// in which `this` level set was specified at construction.
  Vector3<T> CalcLevelSetGradient(const Vector3<T>& p_MQ) const {
    return grad_level_set_M_(p_MQ);
  }

 private:
  std::function<T(const Vector3<T>&)> level_set_M_;
  std::function<Vector3<T>(const Vector3<T>&)> grad_level_set_M_;
};

}  // namespace geometry
}  // namespace drake
