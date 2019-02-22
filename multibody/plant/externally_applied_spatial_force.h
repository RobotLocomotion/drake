#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

template <class T>
struct ExternallyAppliedSpatialForce {
  /// The index of the body that the force is to be applied to.
  BodyIndex body_index;

  /// A position vector from Body B's origin (Bo) to a point Bq (a point of B),
  /// expressed in B's frame.
  Vector3<T> p_BoBq_B;

  /// A spatial force applied to Body B at point Bq, expressed in the
  /// world frame.
  SpatialForce<T> F_Bq_W;
};

}  // namespace multibody
}  // namespace drake
