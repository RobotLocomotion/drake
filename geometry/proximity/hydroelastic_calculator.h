#pragma once

#include <memory>
#include <unordered_map>

#include <fcl/fcl.h>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/hydroelastic_internal.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {

/* Computes ContactSurface using the algorithm appropriate to the Shape types
 represented by the given `compliant` and `rigid` geometries.
 @pre The geometries are not *both* half spaces.  */
template <typename T>
std::unique_ptr<ContactSurface<T>> CalcRigidCompliant(
    const SoftGeometry& soft, const math::RigidTransform<T>& X_WS,
    GeometryId id_S, const RigidGeometry& rigid,
    const math::RigidTransform<T>& X_WR, GeometryId id_R,
    HydroelasticContactRepresentation representation);

/* Computes ContactSurface using the algorithm appropriate to the Shape types
 represented by the given `compliant` geometries.
 @pre None of the geometries are half spaces. */
template <typename T>
std::unique_ptr<ContactSurface<T>> CalcCompliantCompliant(
    const SoftGeometry& compliant_F, const math::RigidTransform<T>& X_WF,
    GeometryId id_F, const SoftGeometry& compliant_G,
    const math::RigidTransform<T>& X_WG, GeometryId id_G,
    HydroelasticContactRepresentation representation);

/* Enumerate the various results of attempting to make a contact surface. */
enum class ContactSurfaceResult {
  /* Computation was successful; either there was no contact (at least one
   geometry was considered vanished), or else the contact surface is guaranteed
   to have at least one face. See also
   `geometry::internal::hydroelastic::Geometries::is_vanished()`. */
  kCalculated,

  /* Contact surface can't be computed for the geometry pair. */
  kUnsupported,

  /* Contact between two half spaces; not allowed. */
  kHalfSpaceHalfSpace,

  /* Contact between two rigid geometries; not allowed. */
  kRigidRigid,

  /* Contact between compliant mesh and compliant half space; not allowed. */
  kCompliantHalfSpaceCompliantMesh,
};

/* @returns true iff the result indicates a failure. Note: If this predicate
 returns false, there might still not be a contact surface. */
bool inline ContactSurfaceFailed(ContactSurfaceResult result) {
  return result != ContactSurfaceResult::kCalculated;
}

/* Calculator for the shape-to-shape hydroelastic contact results. It needs:

    - The T-valued poses of _all_ geometries in the corresponding SceneGraph,
      each indexed by its corresponding geometry's GeometryId.
    - The representation of all geometries that have been prepped for computing
      contact surfaces.
    - The choice of how to represent contact polygons.

 @tparam T The computation scalar.  */
template <typename T>
class ContactCalculator {
 public:
  /* Constructs the fully-specified calculator. The values are as described in
   the class documentation. Some parameters (noted below) are aliased in the
   data and must remain valid at least as long as the ContactCalculator
   instance.

   @param X_WGs                   The T-valued poses. Aliased.
   @param geometries              The set of all hydroelastic geometric
                                  representations. Aliased.
   @param representation          Controls the mesh representation of
                                  the contact surface. See
                                  @ref contact_surface_discrete_representation
                                  "contact surface representation" for more
                                  details. */
  ContactCalculator(
      const std::unordered_map<GeometryId, math::RigidTransform<T>>* X_WGs,
      const Geometries* geometries,
      HydroelasticContactRepresentation representation);

  struct MaybeMakeContactSurfaceResult {
    ContactSurfaceResult result;
    /* Note: `surface` can always be `nullptr`, even when the `result` is
     `kCalculated`. */
    std::unique_ptr<ContactSurface<T>> surface;
  };

  /* Makes the contact surface (if it exists) between two potentially
     colliding geometries.

     @param id_A     Id of the first object in the pair (order insignificant).
     @param id_B     Id of the second object in the pair (order insignificant).
     @returns both the result code, and the new surface, if any. */
  MaybeMakeContactSurfaceResult MaybeMakeContactSurface(GeometryId id_A,
                                                        GeometryId id_B) const;

 private:
  /* The T-valued poses of all geometries.  */
  const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs_;

  /* The hydroelastic geometric representations.  */
  const Geometries& geometries_;

  /* The requested mesh representation type. */
  const HydroelasticContactRepresentation representation_;
};

}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
