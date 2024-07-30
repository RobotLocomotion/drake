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
 represented by the given `soft` and `rigid` geometries.
 @pre The geometries are not *both* half spaces.  */
template <typename T>
std::unique_ptr<ContactSurface<T>> DispatchRigidSoftCalculation(
    const SoftGeometry& soft, const math::RigidTransform<T>& X_WS,
    GeometryId id_S, const RigidGeometry& rigid,
    const math::RigidTransform<T>& X_WR, GeometryId id_R,
    HydroelasticContactRepresentation representation);

/* Computes ContactSurface using the algorithm appropriate to the Shape types
 represented by the given `compliant` geometries.
 @pre None of the geometries are half spaces. */
template <typename T>
std::unique_ptr<ContactSurface<T>> DispatchCompliantCompliantCalculation(
    const SoftGeometry& compliant0_F, const math::RigidTransform<T>& X_WF,
    GeometryId id0, const SoftGeometry& compliant1_G,
    const math::RigidTransform<T>& X_WG, GeometryId id1,
    HydroelasticContactRepresentation representation);

// clang-format off
enum class ContactSurfaceResult {
  kCalculated,  //< Computation was successful; a contact surface is
                //< only produced if the objects were in contact.
  kUnsupported,  //< Contact surface can't be computed for the geometry
                 //< pair.
  kHalfSpaceHalfSpace,  //< Contact between two half spaces; not allowed.
  kRigidRigid,          //< Contact between two rigid geometries; not allowed.
  kCompliantHalfSpaceCompliantMesh,  //< Contact between a compliant mesh and a
                                     //< compliant half space; not allowed.
};
// clang-format on

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
  /* Constructs the fully-specified callback data. The values are as described
   in the class documentation. All parameters are aliased in the data and must
   remain valid at least as long as the CallbackData instance.

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
      HydroelasticContactRepresentation representation)
      : X_WGs_(*X_WGs),
        geometries_(*geometries),
        representation_(representation) {
    DRAKE_DEMAND(X_WGs != nullptr);
    DRAKE_DEMAND(geometries != nullptr);
  }

  struct MaybeMakeContactSurfaceResult {
    ContactSurfaceResult result;
    std::unique_ptr<ContactSurface<T>> surface;
  };

  /* Makes the contact surface (if it exists) between two potentially
     colliding geometries.

     @param id_A     Id of the first object in the pair (order insignificant).
     @param id_B     Id of the second object in the pair (order insignificant).
     @param[out] callback_data   Supporting data to compute the contact surface.
     @returns both the result code, and the new surface, if any.
     @tparam T  The scalar type for the query.  */
  MaybeMakeContactSurfaceResult MaybeMakeContactSurface(GeometryId id_A,
                                                        GeometryId id_B) const;

  /* @throws a std::exception with an appropriate error message for the various
     result codes that indicate failure.
     @pre ContactSurfaceFailed(result) == true
  */
  [[noreturn]] void RejectResult(ContactSurfaceResult result,
                                 fcl::CollisionObjectd* object_A_ptr,
                                 fcl::CollisionObjectd* object_B_ptr) const;

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
