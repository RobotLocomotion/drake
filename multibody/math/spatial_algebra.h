#pragma once

/// @file
/// This is the entry point for all operations with spatial vectors.
/// Spatial vectors represent spatial physical quantities such as spatial
/// velocities, spatial accelerations and spatial forces. Spatial vectors are
/// 6-element quantities that are pairs of ordinary 3-vectors. Elements 0-2 are
/// always the rotational component while elements 3-5 are always the
/// translational component.
/// For a more detailed introduction on spatial vectors please refer to
/// section @ref multibody_spatial_vectors.

#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_acceleration.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/math/spatial_momentum.h"
#include "drake/multibody/math/spatial_velocity.h"

namespace drake {
namespace multibody {

template <typename T>
T SpatialVelocity<T>::dot(const SpatialForce<T>& F) const {
  return this->get_coeffs().dot(F.get_coeffs());
}

template <typename T>
T SpatialForce<T>::dot(const SpatialVelocity<T>& V) const {
  return V.dot(*this);  // dot-product is commutative.
}

template <typename T>
T SpatialMomentum<T>::dot(const SpatialVelocity<T>& V) const {
  return this->get_coeffs().dot(V.get_coeffs());
}

template <typename T>
T SpatialVelocity<T>::dot(const SpatialMomentum<T>& L) const {
  return L.dot(*this);  // dot-product is commutative.
}

}  // namespace multibody
}  // namespace drake
