#pragma once

#include "drake/common/drake_optional.h"
#include "drake/common/unused.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {

/** Computes the contact surface between a compliant sphere and rigid halfspace.
 The halfspace is defined in frame H lying on H's Hx-Hy plane (with its normal
 in the Hz direction). The sphere has the given `radius` and is centered at the
 origin of frame S. The poses of the two geometries are given in the common
 world frame W.

 @note This function does not explicitly interpret the halfspace as volume M or
 N. It depends on the relationship between the ids of the sphere and halfspace.
 The halfspace is interpreted as volume M iff id_H < id_S, otherwise the sphere
 is volume M (see documentation of ContactSurface for details).

 @param radius      The radius of the sphere.
 @param X_WS        The pose of the sphere in the world frame W.
 @param id_S        The geometry id for the sphere.
 @param X_WH        The pose of the halfspace in the world frame W.
 @param id_H        The geometry id for the halfspace.
 @returns The contact surface for the two shapes if the sphere intersects the
 half space. Otherwise `nullopt`.
 @tparam T  The scalar type for the query.  */
template <typename T>
optional<ContactSurface<T>> CalcHalfspaceSphereContact(
    double radius, const math::RigidTransform<T>& X_WS, GeometryId id_S,
    const math::RigidTransform<T>& X_WH, GeometryId id_H) {
  // TODO(amcastro-tri): Write the actual implementation of this method once
  // #11520 and #11519 are in master.
  unused(radius);
  unused(X_WS);
  unused(X_WH);
  // NOTE: Don't read anything into the interpretation of id_S as being id_M;
  // it's just a place holder. See @note in for this method's documentation.
  return ContactSurface<T>(id_S, id_H, nullptr, nullptr, nullptr);
}

}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
