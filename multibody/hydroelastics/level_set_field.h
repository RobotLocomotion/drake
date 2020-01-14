#pragma once

#include <functional>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace hydroelastics {
namespace internal {

// TODO(SeanCurtis-TRI): The contents of this file are no longer used. Refactor
//  this into geometry as appropriate.

// TODO(amcastro-tri): consider making this an abstract class so that we can
// inherit multiple implementations (analytical, structured grids, etc.)
/// This class represents a level set function as the mapping
/// `φ(p_FR): ℝ³ →  ℝ` with `p_FR` the position vector for a point R in a frame
/// F.
template <typename T>
struct LevelSetField {
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LevelSetField)

  /// Constructs a level set field from user-defined functions for a level set
  /// and its gradient.
  /// These functions implicitly defines a frame F for the level set
  /// field. `level_set_F` defines the level set value at a point R from its
  /// position vector `p_FR` in F as `φ(R) ≡ level_set_F(p_FR)`.
  /// Similarly, `grad_level_set_F` is a function taking the position of point R
  /// in frame F, with the resulting gradient expressed in frame F.
  LevelSetField(std::function<T(const Vector3<T>&)> level_set_F,
                std::function<Vector3<T>(const Vector3<T>&)> grad_level_set_F)
      : value(level_set_F), gradient(grad_level_set_F) {}

  std::function<T(const Vector3<T>&)> value;
  std::function<Vector3<T>(const Vector3<T>&)> gradient;
};

}  // namespace internal
}  // namespace hydroelastics
}  // namespace multibody
}  // namespace drake
