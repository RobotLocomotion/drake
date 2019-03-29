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
 * Calculates a tolerance relative to a given `size` parameter with a lower
 * bound of 1e-14 meter. If the `size` parameter is larger than 1 meter, we
 * use the relative tolerance of 1e-14 times the `size`.  If the `size` is
 * smaller than 1 meter, we use the absolute tolerance 1e-14 meter. The
 * 1e-14-meter lower bound helps us handle possible round off errors arising
 * from applying a pose X_WG to a geometry G. Given a query point Q exactly
 * on the boundary ∂G, if we apply X_WG to both Q and G, the point Q is likely
 * to deviate from ∂G more than the machine epsilon, which is around 2e-16.
 */
double DistanceToPointRelativeTolerance(double size);

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
  const Vector3<T> Gx(T(1.), T(0.), T(0.));
  // Gradient vector expressed in G's frame.
  const Vector3<T> grad_G = (dist_GQ > tolerance) ? p_GQ_G / dist_GQ : Gx;

  // Do not compute distance as |p_GQ|, as the gradient of |p_GQ| is not well
  // defined at p_GQ = 0. Instead, compute it as p_GQ.dot(grad_G).
  *distance = p_GQ_G.dot(grad_G) - T(radius);

  // Position vector of the nearest point N on G's surface from the query
  // point Q, expressed in G's frame.
  *p_GN = radius * grad_G;
  // Gradient vector expressed in World frame.
  *grad_W = X_WG.rotation() * grad_G;
}

template <typename T>
void ComputeDistanceToPrimitive(const fcl::Halfspaced& halfspace,
                                const math::RigidTransform<T>& X_WG,
                                const Vector3<T>& p_WQ, Vector3<T>* p_GN,
                                T* distance, Vector3<T>* grad_W) {
  // FCL stores the halfspace as {x | nᵀ * x <= d}, with n being a unit length
  // normal vector.
  const Vector3<T> p_GQ = X_WG.inverse() * p_WQ;
  *distance = halfspace.n.cast<T>().dot(p_GQ) - T(halfspace.d);
  *p_GN = p_GQ - *distance * halfspace.n.cast<T>();
  *grad_W = (X_WG.rotation() * halfspace.n).template cast<T>();
}
/**
 * An internal functor to support the call back function in proximity_engine.cc.
 * It computes the signed distance to a query point from a supported geometry.
 */
class DistanceToPointWithGradient {
 public:
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
