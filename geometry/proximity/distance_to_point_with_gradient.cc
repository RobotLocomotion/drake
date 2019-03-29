#include "drake/geometry/proximity/distance_to_point_with_gradient.h"

#include <algorithm>

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
    p_WQ_autodiff(i).derivatives() = X_WG_.rotation().row(i).transpose();
  }
  X_WG_autodiff.set_rotation(X_WG_.rotation().cast<AutoDiffd<3>>());
  X_WG_autodiff.set_translation(X_WG_.translation().cast<AutoDiffd<3>>());
  Vector3<AutoDiffd<3>> p_GN_autodiff, grad_W_autodiff;
  AutoDiffd<3> distance_autodiff;
  ComputeDistanceToPrimitive(primitive, X_WG_autodiff, p_WQ_autodiff,
                             &p_GN_autodiff, &distance_autodiff,
                             &grad_W_autodiff);
  SignedDistanceToPointWithGradient signed_distance;
  signed_distance.id_G = geometry_id_;
  signed_distance.p_GN = math::autoDiffToValueMatrix(p_GN_autodiff);
  signed_distance.dp_GN_dp_GQ = math::autoDiffToGradientMatrix(p_GN_autodiff);
  signed_distance.distance = distance_autodiff.value();
  signed_distance.ddistance_dp_GQ = distance_autodiff.derivatives().transpose();
  signed_distance.grad_W = math::autoDiffToValueMatrix(grad_W_autodiff);
  signed_distance.dgrad_W_dp_GQ =
      math::autoDiffToGradientMatrix(grad_W_autodiff);
  return signed_distance;
}

double DistanceToPointRelativeTolerance(double size) {
  return 1e-14 * std::max(1., size);
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
