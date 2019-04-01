#pragma once

#include <utility>

#include <fcl/geometry/shape/halfspace.h>
#include <fcl/geometry/shape/sphere.h>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_results/signed_distance_to_point_with_gradient.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
/**
 * Calculates an absolute tolerance value conditioned to a problem's
 * characteristic `size`. The tolerance is sufficient to account for round-off
 * error which arises due to transformations.
 */
double DistanceToPointRelativeTolerance(double size);

/**
 * @name ComputeDistanceToPrimitive
 * Computes the signed distance from a point Q to a primitive.
 * Refer to QueryObject::ComputeSignedDistanceToPoint for more details.
 * @param X_WG The pose of the primitive geometry G in the world frame.
 * @param p_WQ The position of the point Q in the world frame.
 * @param p_GN[out] The position of the witness point N on the primitive,
 * expressed in the primitive geometry frame G.
 * @param distance[out] The signed distance from the point Q to the primitive.
 * @param grad_W[out] The gradient of the distance function w.r.t p_GQ (the
 * position of the point Q in the primitiveframe). This gradient vector is
 * expressed in the world frame.
 * @{
 */

/** Overload of ComputeDistanceToPrimitive() for sphere primitive. */
template <typename T>
void ComputeDistanceToPrimitive(const fcl::Sphered& sphere,
                                const math::RigidTransform<T>& X_WG,
                                const Vector3<T>& p_WQ, Vector3<T>* p_GN,
                                T* distance, Vector3<T>* grad_W) {
  // TODO(DamrongGuoy): Move most code of this function into FCL.
  const double radius = sphere.radius;
  const Vector3<T> p_GQ_G = X_WG.inverse() * p_WQ;
  const T dist_GQ = p_GQ_G.norm();

  // The gradient is always in the direction from the center of the sphere to
  // the query point Q, regardless of whether the point Q is outside or inside
  // the sphere G.  The gradient is undefined if the query point Q is at the
  // center of the sphere G.
  //
  // If the query point Q is near the center of the sphere G within a
  // tolerance, we arbitrarily set the gradient vector as documented in
  // query_object.h (QueryObject::ComputeSignedDistanceToPoint).
  const double tolerance = DistanceToPointRelativeTolerance(radius);
  // Unit vector in x-direction of G's frame.
  const Vector3<T> Gx = Vector3<T>::UnitX();
  // Gradient vector expressed in G's frame.
  const Vector3<T> grad_G = (dist_GQ > tolerance) ? p_GQ_G / dist_GQ : Gx;

  // Do not compute distance as |p_GQ|, as the gradient of |p_GQ| w.r.t p_GQ is
  // p_GQᵀ/|p_GQ|, not well defined at p_GQ = 0. Instead, compute the distance
  // as p_GQ.dot(grad_G).
  *distance = p_GQ_G.dot(grad_G) - T(radius);

  // Position vector of the nearest point N on G's surface from the query
  // point Q, expressed in G's frame.
  *p_GN = radius * grad_G;
  // Gradient vector expressed in World frame.
  *grad_W = X_WG.rotation() * grad_G;
}

/** Overload of ComputeDistanceToPrimitive() for halfspace primitive. */
template <typename T>
void ComputeDistanceToPrimitive(const fcl::Halfspaced& halfspace,
                                const math::RigidTransform<T>& X_WG,
                                const Vector3<T>& p_WQ, Vector3<T>* p_GN,
                                T* distance, Vector3<T>* grad_W) {
  // FCL stores the halfspace as {x | nᵀ * x <= d}, with n being a unit length
  // normal vector. Both n and x are expressed in the halfspace frame.
  const Vector3<T> n_G = halfspace.n.cast<T>();
  const Vector3<T> p_GQ = X_WG.inverse() * p_WQ;
  *distance = n_G.dot(p_GQ) - T(halfspace.d);
  *p_GN = p_GQ - *distance * n_G;
  *grad_W = X_WG.rotation() * n_G;
}
//@}

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

  /** Overload for halfspace object */
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
