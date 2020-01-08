#pragma once

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <fmt/format.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/collision_filter_legacy.h"
#include "drake/geometry/proximity/hydroelastic_internal.h"
#include "drake/geometry/proximity/mesh_intersection.h"
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

    - A collision filter instance.
    - The T-valued poses of _all_ geometries in the corresponding SceneGraph,
      each indexed by its corresponding geometry's GeometryId.
    - The representation of all geometries that have been prepped for computing
      contact surfaces.
    - A vector of contact surfaces -- one instance of ContactSurface for
      every supported, unfiltered penetrating pair.

 @tparam T The computation scalar.  */
template <typename T>
struct CallbackData {
  /** Constructs the fully-specified callback data. The values are as described
   in the class documentation. The parameters are all aliased in the data and
   must remain valid at least as long as the CallbackData instance.

   @param collision_filter_in     The collision filter system. Aliased.
   @param X_WGs_in                The T-valued poses. Aliased.
   @param geometries_in           The set of all hydroelastic geometric
                                  representations. Aliased.
   @param surfaces_in             The output results. Aliased.  */
  CallbackData(
      const CollisionFilterLegacy* collision_filter_in,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>* X_WGs_in,
      const Geometries* geometries_in,
      std::vector<ContactSurface<T>>* surfaces_in)
      : collision_filter(*collision_filter_in),
        X_WGs(*X_WGs_in),
        geometries(*geometries_in),
        surfaces(*surfaces_in) {
    DRAKE_DEMAND(collision_filter_in);
    DRAKE_DEMAND(X_WGs_in);
    DRAKE_DEMAND(geometries_in);
    DRAKE_DEMAND(surfaces_in);
  }

  /** The collision filter system.  */
  const CollisionFilterLegacy& collision_filter;

  /** The T-valued poses of all geometries.  */
  const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs;

  /** The hydroelastic geometric representations.  */
  const Geometries& geometries;

  /** The results of the distance query.  */
  std::vector<ContactSurface<T>>& surfaces;
};

/** The callback function for computing a hydroelastic contact surface between
 two arbitrary shapes.

 @param object_A_ptr    Pointer to the first object in the pair (the order has
                        no significance).
 @param object_B_ptr    Pointer to the second object in the pair (the order has
                        no significance).
 @param callback_data   Supporting data to compute the contact surface.
 @returns false; the broadphase should *not* terminate its process.
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
    const HydroelasticType type_A =
        data.geometries.hydroelastic_type(encoding_a.id());
    const HydroelasticType type_B =
        data.geometries.hydroelastic_type(encoding_b.id());
    if (type_A == HydroelasticType::kUndefined ||
        type_B == HydroelasticType::kUndefined) {
      throw std::logic_error(fmt::format(
          "Requested a contact surface between a pair of geometries without "
          "hydroelastic representation for at least one shape: a {} {} with id "
          "{} and a {} {} with id {}",
          type_A, GetGeometryName(*object_A_ptr), encoding_a.id(), type_B,
          GetGeometryName(*object_B_ptr), encoding_b.id()));
    }
    if (type_A == type_B) {
      throw std::logic_error(fmt::format(
          "Requested contact between two {} objects ({} with id "
          "{}, {} with id {}); only rigid-soft pairs are currently supported",
          type_A, GetGeometryName(*object_A_ptr), encoding_a.id(),
          GetGeometryName(*object_B_ptr), encoding_b.id()));
    }

    const bool A_is_rigid = type_A == HydroelasticType::kRigid;
    const GeometryId id_S = A_is_rigid ? encoding_b.id() : encoding_a.id();
    const GeometryId id_R = A_is_rigid ? encoding_a.id() : encoding_b.id();

    const math::RigidTransform<T>& X_WS(data.X_WGs.at(id_S));
    const math::RigidTransform<T>& X_WR(data.X_WGs.at(id_R));

    // TODO(SeanCurtis-TRI): We are currently assuming that *everything* is a
    //  mesh. When rigid and compliant half spaces are fully supported modify
    //  this.
    const SoftGeometry& soft = data.geometries.soft_geometry(id_S);
    const VolumeMeshField<double, double>& field_S = soft.pressure_field();
    const BoundingVolumeHierarchy<VolumeMesh<double>>& bvh_S = soft.bvh();
    const RigidGeometry& rigid = data.geometries.rigid_geometry(id_R);
    const SurfaceMesh<double>& mesh_R = rigid.mesh();
    const BoundingVolumeHierarchy<SurfaceMesh<double>>& bvh_R = rigid.bvh();

    // TODO(SeanCurtis-TRI): There are multiple heap allocations implicit in
    // this (resizing vector, constructing mesh and pressure field), this *may*
    // prove to be too expensive to be in the inner loop of the simulation.
    // Keep an eye on this.
    std::unique_ptr<ContactSurface<T>> surface =
        ComputeContactSurfaceFromSoftVolumeRigidSurface(
            id_S, field_S, bvh_S, X_WS, id_R, mesh_R, bvh_R, X_WR);
    if (surface != nullptr) {
      DRAKE_DEMAND(surface->id_M() < surface->id_N());
      data.surfaces.emplace_back(std::move(*surface));
    }
  }
  // Tell the broadphase to keep searching.
  return false;
}

}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
