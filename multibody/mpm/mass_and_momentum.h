#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
/* The mass, momentum, and angular momentum of a system (expressed in the world
 frame where applicable). The angular momentum is about the world origin. */
template <typename T>
struct MassAndMomentum {
  T mass{0.0};
  Vector3<T> linear_momentum{Vector3<T>::Zero()};
  Vector3<T> angular_momentum{Vector3<T>::Zero()};
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
