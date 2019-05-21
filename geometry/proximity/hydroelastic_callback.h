#pragma once

#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <fmt/format.h>

#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/collision_filter_legacy.h"
#include "drake/geometry/proximity/hydroelastic_sphere_halfspace.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {

/** Supporting data for the shape-to-shape hydroelastic contact callback (see
 Callback below). It includes:

    - A map from GeometryIndex to GeometryId (to facilitate reporting GeometryId
      values in the results).
    - A collision filter instance.
    - The T-valued poses of _all_ geometries in the corresponding SceneGraph,
      each indexed by its corresponding geometry's GeometryIndex.
    - A vector of contact surfaces -- one instance of ContactSurface for
      every supported, unfiltered penetrating pair.

 @tparam T The computation scalar.  */
template <typename T>
struct CallbackData {
  /** Constructs the fully-specified callback data. The values are as described
   in the class documentation. The parameters are all aliased in the data and
   must remain valid at least as long as the %CallbackData instance.

   @param geometry_map_in         The index -> id map. Aliased.
   @param collision_filter_in     The collision filter system. Aliased.
   @param X_WGs_in                The T-valued poses. Aliased.
   @param surfaces_in             The output results. Aliased.  */
  CallbackData(const std::vector<GeometryId>* geometry_map_in,
               const CollisionFilterLegacy* collision_filter_in,
               const std::vector<Isometry3<T>>* X_WGs_in,
               std::vector<ContactSurface<T>>* surfaces_in)
      : geometry_map(*geometry_map_in),
        collision_filter(*collision_filter_in),
        X_WGs(*X_WGs_in),
        surfaces(*surfaces_in) {
    DRAKE_DEMAND(geometry_map_in);
    DRAKE_DEMAND(collision_filter_in);
    DRAKE_DEMAND(X_WGs_in);
    DRAKE_DEMAND(surfaces_in);
  }

  /** The map from GeometryIndex to GeometryId.  */
  const std::vector<GeometryId>& geometry_map;

  /** The collision filter system.  */
  const CollisionFilterLegacy& collision_filter;

  /** The T-valued poses of all geometries.  */
  const std::vector<Isometry3<T>>& X_WGs;

  /** The results of the distance query.  */
  std::vector<ContactSurface<T>>& surfaces{};
};

// TODO(SeanCurtis-TRI): Replace this clunky mechanism with a new mechanism
// which does this implicitly via ADL and templates.
/** @name   Mechanism for reporting on which scalars and for which shape-pairs
            contact surface queries can be made.

 By default, *only* sphere-halfspace is supported, but for all scalars.

 @tparam T      The computational scalar type.  */
//@{

template <typename T>
struct ScalarSupport {
  static bool is_supported(fcl::NODE_TYPE node1, fcl::NODE_TYPE node2) {
    // Current version only supports sphere-halfspace contact.
    return (node1 == fcl::GEOM_HALFSPACE && node2 == fcl::GEOM_SPHERE) ||
           (node2 == fcl::GEOM_HALFSPACE && node1 == fcl::GEOM_SPHERE);
  }
};

//@}

/** The callback function for computing a hydroelastic contact surface between
 two arbitrary shapes.

 @param object_A_ptr    Pointer to the first object in the pair (the order has
                        no significance).
 @param object_B_ptr    Pointer to the second object in the pair (the order has
                        no significance).
 @param callback_data   Supporting data to compute the contact surface.
 @returns False; the broadphase should *not* terminate its process.
 @tparam T  The scalar type for the query.  */
template <typename T>
bool Callback(fcl::CollisionObjectd* object_A_ptr,
              fcl::CollisionObjectd* object_B_ptr,
              // NOLINTNEXTLINE
              void* callback_data) {
  auto& data = *static_cast<CallbackData<T>*>(callback_data);

  const EncodedData encoding_a(*object_A_ptr);
  const EncodedData encoding_b(*object_B_ptr);

  const bool can_collide = data.collision_filter.CanCollideWith(
      encoding_a.encoding(), encoding_b.encoding());

  if (can_collide) {
    if (ScalarSupport<T>::is_supported(
            object_A_ptr->collisionGeometry()->getNodeType(),
            object_B_ptr->collisionGeometry()->getNodeType())) {
      // NOTE: This A_is_sphere only works because right now, we only support
      // sphere-halfspace. When we support a larger space, we'll have to be more
      // clever about this.
      bool A_is_sphere =
          object_A_ptr->collisionGeometry()->getNodeType() == fcl::GEOM_SPHERE;
      const GeometryId id_S = A_is_sphere ? encoding_a.id(data.geometry_map)
                                          : encoding_b.id(data.geometry_map);
      const GeometryId id_H = A_is_sphere ? encoding_b.id(data.geometry_map)
                                          : encoding_a.id(data.geometry_map);
      const GeometryIndex index_S =
          A_is_sphere ? encoding_a.index() : encoding_b.index();
      const GeometryIndex index_H =
          A_is_sphere ? encoding_b.index() : encoding_a.index();
      double radius = A_is_sphere ? static_cast<const fcl::Sphered*>(
                                        object_A_ptr->collisionGeometry().get())
                                        ->radius
                                  : static_cast<const fcl::Sphered*>(
                                        object_B_ptr->collisionGeometry().get())
                                        ->radius;
      const math::RigidTransform<T> X_WS(data.X_WGs[index_S]);
      const math::RigidTransform<T> X_WH(data.X_WGs[index_H]);
      optional<ContactSurface<T>> surface_maybe =
          CalcHalfspaceSphereContact(radius, X_WS, id_S, X_WH, id_H);
      //  Just because the broadphase says these two might be colliding, doesn't
      //  mean they are colliding. We verify that.
      if (surface_maybe) {
        DRAKE_DEMAND(surface_maybe->id_M() < surface_maybe->id_N());
        data.surfaces.emplace_back(std::move(*surface_maybe));
      }
    } else {
      throw std::logic_error(
          fmt::format("Contact surface queries between shapes '{}' and '{}' "
                      "are not supported for scalar type {}",
                      GetGeometryName(*object_A_ptr),
                      GetGeometryName(*object_B_ptr), NiceTypeName::Get<T>()));
    }
  }
  // Tell the broadphase to keep searching.
  return false;
}

}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
