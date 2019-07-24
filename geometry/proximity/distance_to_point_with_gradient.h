#pragma once

#include <utility>

#include <fcl/geometry/shape/halfspace.h>
#include <fcl/geometry/shape/sphere.h>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/query_results/signed_distance_to_point_with_gradient.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/**
 * An internal functor to support the call back function in proximity_engine.cc.
 * It computes the signed distance to a query point from a supported geometry.
 */
class DistanceToPointWithGradient {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DistanceToPointWithGradient)

  DistanceToPointWithGradient(const GeometryId id, math::RigidTransformd X_WG,
                              Eigen::Vector3d p_WQ)
      : geometry_id_(id), X_WG_(std::move(X_WG)), p_WQ_(std::move(p_WQ)) {}

  /** Overload for sphere object */
  SignedDistanceToPointWithGradient operator()(
      const fcl::Sphered& sphere) const;

  /** Overload for half space object */
  SignedDistanceToPointWithGradient operator()(
      const fcl::Halfspaced& halfspace) const;

 private:
  template <typename PrimitiveType>
  SignedDistanceToPointWithGradient ComputeDistance(
      const PrimitiveType& primitive) const;

  const GeometryId geometry_id_;
  const math::RigidTransformd X_WG_;
  const Eigen::Vector3d p_WQ_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
