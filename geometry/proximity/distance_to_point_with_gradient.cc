#include "drake/geometry/proximity/distance_to_point_with_gradient.h"

#include <algorithm>

#include "drake/geometry/proximity/distance_to_point.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename PrimitiveType>
SignedDistanceToPointWithGradient DistanceToPointWithGradient::ComputeDistance(
    const PrimitiveType& primitive) const {
  // p_WQ = X_WG * p_GQ. Hence ∂p_WQ / ∂p_GQ = R_WG, ∂X_WG / ∂p_GQ = 0.
  math::RigidTransform<AutoDiffd<3>> X_WG_autodiff;
  Vector3<AutoDiffd<3>> p_WQ_autodiff;
  for (int i = 0; i < 3; ++i) {
    p_WQ_autodiff(i).value() = p_WQ_(i);
    // derivatives() is a column vector. Hence we need the transpose on the
    // right-hand side.
    p_WQ_autodiff(i).derivatives() = X_WG_.rotation().row(i).transpose();
  }
  X_WG_autodiff.set_rotation(X_WG_.rotation().cast<AutoDiffd<3>>());
  X_WG_autodiff.set_translation(X_WG_.translation().cast<AutoDiffd<3>>());
  Vector3<AutoDiffd<3>> p_GN_autodiff, grad_W_autodiff;
  AutoDiffd<3> distance_autodiff;
  bool is_grad_W_well_defined{};
  point_distance::ComputeDistanceToPrimitive(
      primitive, X_WG_autodiff, p_WQ_autodiff, &p_GN_autodiff,
      &distance_autodiff, &grad_W_autodiff, &is_grad_W_well_defined);
  return SignedDistanceToPointWithGradient(
      geometry_id_, math::autoDiffToValueMatrix(p_GN_autodiff),
      math::autoDiffToGradientMatrix(p_GN_autodiff), distance_autodiff.value(),
      distance_autodiff.derivatives().transpose(),
      math::autoDiffToValueMatrix(grad_W_autodiff),
      math::autoDiffToGradientMatrix(grad_W_autodiff), is_grad_W_well_defined);
}

SignedDistanceToPointWithGradient DistanceToPointWithGradient::operator()(
    const fcl::Sphered& sphere) const {
  return ComputeDistance(sphere);
}

SignedDistanceToPointWithGradient DistanceToPointWithGradient::operator()(
    const fcl::Halfspaced& halfspace) const {
  return ComputeDistance(halfspace);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
