#pragma once

#include <functional>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {

/// This class represents a level set function as the application
/// `φ(ᴹpᴿ): ℝ³ →  ℝ` with `ᴹpᴿ` the position vector for a point R in a frame M.
// TODO(amcastro-tri): consider making this an abstract class so that we can
// inherit multiple implementations (analytical, structured grids, etc.)
template <typename T>
class LevelSetField {
 public:
  /// Constructs a level set field from `std::function` objects specifing both
  /// the level set function `level_set_M` as well as its gradient
  /// `grad_level_set_M`.
  /// These functions implicitly define the model frame M for the level set
  /// field. `level_set_M` defines the level set function at a point R from its
  /// position vector `p_MR` in M as `φ(R) ≡ level_set_N(p_MR)`.
  /// Similarly, `grad_level_set_M` is a function taking the position of point R
  /// in model frame M, with the resulting gradient expressed in frame M.
  LevelSetField(std::function<T(const Vector3<T>&)> level_set_M,
                std::function<Vector3<T>(const Vector3<T>&)> grad_level_set_M)
      : level_set_M_(level_set_M), grad_level_set_M_(grad_level_set_M) {}

  LevelSetField(LevelSetField&&) = default;
  LevelSetField& operator=(LevelSetField&&) = default;

  /// Returns the the underlying `std::function` to the level set function.
  const std::function<T(const Vector3<T>&)>& level_set_M() const {
    return level_set_M_;
  }

  /// Returns the the underlying `std::function` to the gradient of the level
  /// set function.
  const std::function<Vector3<T>(const Vector3<T>&)>& grad_level_set_M() const {
    return grad_level_set_M_;
  }

  /// Computes the level set at a point Q measured and expressed in the model
  /// frame M as defined at construction.
  T CalcLevelSet(const Vector3<T>& p_MQ) const {
    return level_set_M_(p_MQ);
  }

  /// Computes the gradient of the level set at a point Q measured and expressed
  /// in the model frame M as defined at construction. The result is expressed
  /// in frame M.
  Vector3<T> CalcLevelSetGradient(const Vector3<T>& p_MQ) const {
    return grad_level_set_M_(p_MQ);
  }

 private:
  std::function<T(const Vector3<T>&)> level_set_M_;
  std::function<Vector3<T>(const Vector3<T>&)> grad_level_set_M_;
};

}  // namespace geometry
}  // namespace drake
