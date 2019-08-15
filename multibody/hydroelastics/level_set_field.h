#pragma once

#include <functional>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace hydroelastics {
namespace internal {

// TODO(amcastro-tri): consider making this an abstract class so that we can
// inherit multiple implementations (analytical, structured grids, etc.)
/// This class represents a level set function as the mapping
/// `φ(p_WR): ℝ³ →  ℝ` with `p_WR` the position vector for a point R in the
/// global frame W.
template <typename T>
struct LevelSetField {
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LevelSetField)

  /// Constructs a level set field from user-defined functions for a level set
  /// and its gradient.
  /// These functions implicitly defines a frame F for the level set
  /// field. `level_set_W` defines the level set value at a point R from its
  /// position vector `p_WR` in the world frame as `φ(R) ≡ level_set_F(p_WR)`.
  /// Similarly, `grad_level_set_W` is a function taking the position of point R
  /// in the world frame, with the resulting gradient expressed in the world
  /// frame also.
  LevelSetField(std::function<T(const Vector3<T>&)> level_set_W,
                std::function<Vector3<T>(const Vector3<T>&)> grad_level_set_W)
      : value(level_set_W), gradient(grad_level_set_W) {}

  std::function<T(const Vector3<T>&)> value;
  std::function<Vector3<T>(const Vector3<T>&)> gradient;
};

}  // namespace internal
}  // namespace hydroelastics
}  // namespace multibody
}  // namespace drake
