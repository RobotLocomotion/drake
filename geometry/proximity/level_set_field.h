#pragma once

#include <functional>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {

/// This class represents a level set function as the mapping
/// `φ(ᴹpᴿ): ℝ³ →  ℝ` with `ᴹpᴿ` the position vector for a point R in a frame M.
// TODO(amcastro-tri): consider making this an abstract class so that we can
// inherit multiple implementations (analytical, structured grids, etc.)
template <typename T>
class LevelSetField {
 public:
  /// Constructs a level set field from user-defined functions for a level set
  /// and its gradient.
  /// These functions implicitly defines a frame F for the level set
  /// field. `level_set_F` defines the level set value at a point R from its
  /// position vector `p_FR` in F as `φ(R) ≡ level_set_F(p_FR)`.
  /// Similarly, `grad_level_set_F` is a function taking the position of point R
  /// in frame F, with the resulting gradient expressed in frame F.
  LevelSetField(std::function<T(const Vector3<T>&)> level_set_F,
                std::function<Vector3<T>(const Vector3<T>&)> grad_level_set_F)
      : level_set_F_(level_set_F), grad_level_set_F_(grad_level_set_F) {}

  LevelSetField(LevelSetField&&) = default;
  LevelSetField& operator=(LevelSetField&&) = default;

  /// Returns the the underlying level set function.
  const std::function<T(const Vector3<T>&)>& level_set_F() const {
    return level_set_F_;
  }

  /// Returns the the underlying `std::function` to the gradient of the level
  /// set function.
  const std::function<Vector3<T>(const Vector3<T>&)>& grad_level_set_F() const {
    return grad_level_set_F_;
  }

  /// Computes the level set at a point Q measured and expressed in the frame F
  /// as defined at construction.
  T CalcLevelSet(const Vector3<T>& p_FQ) const {
    return level_set_F_(p_FQ);
  }

  /// Computes the gradient of the level set at a point Q measured and expressed
  /// in the model frame F as defined at construction. The result is expressed
  /// in frame F.
  Vector3<T> CalcLevelSetGradient(const Vector3<T>& p_FQ) const {
    return grad_level_set_F_(p_FQ);
  }

 private:
  std::function<T(const Vector3<T>&)> level_set_F_;
  std::function<Vector3<T>(const Vector3<T>&)> grad_level_set_F_;
};

}  // namespace geometry
}  // namespace drake
