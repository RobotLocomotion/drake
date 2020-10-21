#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fem {
/** The constant parameters used in the element routine FemElasticity. */
template <typename T>
struct FemElasticityParameters {
  // TODO(xuchenhan-tri): Add timestep when dynamic elasticity is introduced in
  // FemElasticity.
  // TODO(xuchenhan-tri): Add damping model.
  Matrix3X<T> reference_positions;
  Vector3<T> gravity{0, 0, -9.81};
  T density{0};
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
