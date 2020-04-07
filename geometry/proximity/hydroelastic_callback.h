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
#include "drake/geometry/proximity/mesh_half_space_intersection.h"
#include "drake/geometry/proximity/mesh_intersection.h"
#include "drake/geometry/proximity/mesh_plane_intersection.h"
#include "drake/geometry/proximity/penetration_as_point_pair_callback.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
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

enum class CalcContactSurfaceResult {
  kCalculated,          //< Computation was successful; a contact surface is
                        //< only produced if the objects were in contact.
  kUnsupported,         //< Contact surface can't be computed for the geometry
                        //< pair.
  kHalfSpaceHalfSpace,  //< Contact between two half spaces; not allowed.
  kSameCompliance       //< The two geometries have the same compliance type.
};

/** Computes ContactSurface using the algorithm appropriate to the Shape types
 represented by the given `soft` and `rigid` geometries.
 @pre The geometries are not *both* half spaces.  */
template <typename T>
std::unique_ptr<ContactSurface<T>> DispatchRigidSoftCalculation(
    const SoftGeometry& soft, const math::RigidTransform<T>& X_WS,
    GeometryId id_S, const RigidGeometry& rigid,
    const math::RigidTransform<T>& X_WR, GeometryId id_R) {
  if (soft.is_half_space() || rigid.is_half_space()) {
    if (soft.is_half_space()) {
      DRAKE_DEMAND(!rigid.is_half_space());
      // Soft half space with rigid mesh.
      const SurfaceMesh<double>& mesh_R = rigid.mesh();
      const BoundingVolumeHierarchy<SurfaceMesh<double>>& bvh_R = rigid.bvh();

      return ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
          id_S, X_WS, soft.pressure_scale(), id_R, mesh_R, bvh_R, X_WR);
    } else {
      // Soft volume vs rigid half space. The half space-mesh intersection
      // requires the mesh field to be a linear mesh field.
      const auto& field_S =
          dynamic_cast<const VolumeMeshFieldLinear<double, double>&>(
              soft.pressure_field());
      const BoundingVolumeHierarchy<VolumeMesh<double>>& bvh_S = soft.bvh();
      return ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(
          id_S, field_S, bvh_S, X_WS, id_R, X_WR);
    }
  } else {
    // soft cannot be a half space; so this must be mesh-mesh.
    const VolumeMeshField<double, double>& field_S = soft.pressure_field();
    const BoundingVolumeHierarchy<VolumeMesh<double>>& bvh_S = soft.bvh();
    const SurfaceMesh<double>& mesh_R = rigid.mesh();
    const BoundingVolumeHierarchy<SurfaceMesh<double>>& bvh_R = rigid.bvh();

    return ComputeContactSurfaceFromSoftVolumeRigidSurface(
        id_S, field_S, bvh_S, X_WS, id_R, mesh_R, bvh_R, X_WR);
  }
}

/** Calculates the contact surface (if it exists) between two potentially
 colliding geometries.

 @param object_A_ptr         Pointer to the first object in the pair (the order
                             has no significance).
 @param object_B_ptr         Pointer to the second object in the pair (the order
                             has no significance).
 @param[out] callback_data   Supporting data to compute the contact surface. If
                             the objects collide, a ContactSurface instance will
                             be added to the data.
 @returns The result from attempting to perform the calculation (indicating if
          it was supported or not -- and in what way).
 @tparam T  The scalar type for the query.  */
template <typename T>
CalcContactSurfaceResult MaybeCalcContactSurface(
    fcl::CollisionObjectd* object_A_ptr, fcl::CollisionObjectd* object_B_ptr,
    CallbackData<T>* data) {
  const EncodedData encoding_a(*object_A_ptr);
  const EncodedData encoding_b(*object_B_ptr);

  const HydroelasticType type_A =
      data->geometries.hydroelastic_type(encoding_a.id());
  const HydroelasticType type_B =
      data->geometries.hydroelastic_type(encoding_b.id());
  if (type_A == HydroelasticType::kUndefined ||
      type_B == HydroelasticType::kUndefined) {
    return CalcContactSurfaceResult::kUnsupported;
  }
  if (type_A == type_B) {
    return CalcContactSurfaceResult::kSameCompliance;
  }

  bool A_is_rigid = type_A == HydroelasticType::kRigid;
  const GeometryId id_S = A_is_rigid ? encoding_b.id() : encoding_a.id();
  const GeometryId id_R = A_is_rigid ? encoding_a.id() : encoding_b.id();

  const SoftGeometry& soft = data->geometries.soft_geometry(id_S);
  const RigidGeometry& rigid = data->geometries.rigid_geometry(id_R);

  if (soft.is_half_space() && rigid.is_half_space()) {
    return CalcContactSurfaceResult::kHalfSpaceHalfSpace;
  }

  const math::RigidTransform<T>& X_WS(data->X_WGs.at(id_S));
  const math::RigidTransform<T>& X_WR(data->X_WGs.at(id_R));

  std::unique_ptr<ContactSurface<T>> surface =
      DispatchRigidSoftCalculation(soft, X_WS, id_S, rigid, X_WR, id_R);

  if (surface != nullptr) {
    DRAKE_DEMAND(surface->id_M() < surface->id_N());
    data->surfaces.emplace_back(std::move(*surface));
  }

  return CalcContactSurfaceResult::kCalculated;
}

/** Assess contact between two objects -- if it can't be determined with
 hydroelastic contact, it throws an exception. All parameters are as documented
 in MaybeCalcContactSurface().
 @returns `false`; the broad phase should _not_ terminate its process.
 @pre `callback_data` must be an instance of CallbackData.  */
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
    CalcContactSurfaceResult result =
        MaybeCalcContactSurface(object_A_ptr, object_B_ptr, &data);

    // Surface calculated; we're done.
    if (result == CalcContactSurfaceResult::kCalculated) return false;

    const HydroelasticType type_A =
        data.geometries.hydroelastic_type(encoding_a.id());
    const HydroelasticType type_B =
        data.geometries.hydroelastic_type(encoding_b.id());

    switch (result) {
      case CalcContactSurfaceResult::kUnsupported:
        throw std::logic_error(fmt::format(
            "Requested a contact surface between a pair of geometries without "
            "hydroelastic representation for at least one shape: a {} {} with "
            "id {} and a {} {} with id {}",
            type_A, GetGeometryName(*object_A_ptr), encoding_a.id(), type_B,
            GetGeometryName(*object_B_ptr), encoding_b.id()));
      case CalcContactSurfaceResult::kSameCompliance:
        throw std::logic_error(fmt::format(
            "Requested contact between two {} objects ({} with id "
            "{}, {} with id {}); only rigid-soft pairs are currently supported",
            type_A, GetGeometryName(*object_A_ptr), encoding_a.id(),
            GetGeometryName(*object_B_ptr), encoding_b.id()));
      case CalcContactSurfaceResult::kHalfSpaceHalfSpace:
        throw std::logic_error(fmt::format(
            "Requested contact between two half spaces with ids {} and {}; "
            "that is not allowed", encoding_a.id(), encoding_b.id()));
      default:
        DRAKE_UNREACHABLE();
    }
  }

  // Tell the broadphase to keep searching.
  return false;
}

/** Supporting data for the shape-to-shape hydroelastic contact callback with
 fallback (see CallbackWithFallback below). It includes:

    - CallbackData for strict hydroelastic contact (see above).
    - A vector of contacts represented as point pairs -- comprising of those
      contacts which could not be evaluated with hydroelastic models.

 @tparam T The computation scalar.  */
template <typename T>
struct CallbackWithFallbackData {
  CallbackData<T> data;
  std::vector<PenetrationAsPointPair<double>>* point_pairs;
};

/** Assess contact between two objects -- if it can't be determined with
 hydroelastic contact, it assess the contact using point-contact. All parameters
 are as documented in MaybeCalcContactSurface(). However, both ContactSurface
 and PenetrationAsPointPair instances are written to the `callback_data`.

 @returns `false`; the broad phase should _not_ terminate its process.
 @pre `callback_data` must be an instance of CallbackWithFallbackData.  */
template <typename T>
bool CallbackWithFallback(fcl::CollisionObjectd* object_A_ptr,
                          fcl::CollisionObjectd* object_B_ptr,
                          // NOLINTNEXTLINE
                          void* callback_data) {
  auto& data = *static_cast<CallbackWithFallbackData<T>*>(callback_data);

  const EncodedData encoding_a(*object_A_ptr);
  const EncodedData encoding_b(*object_B_ptr);

  const bool can_collide = data.data.collision_filter.CanCollideWith(
      encoding_a.encoding(), encoding_b.encoding());

  if (can_collide) {
    CalcContactSurfaceResult result =
        MaybeCalcContactSurface(object_A_ptr, object_B_ptr, &data.data);

    // Surface calculated; we're done.
    if (result == CalcContactSurfaceResult::kCalculated) return false;

    // Fall back to point pair.
    // TODO(SeanCurtis-TRI): This is a problem; point pair is only double.
    //   fallback can only be double.
    penetration_as_point_pair::CallbackData point_data{
        &data.data.collision_filter, data.point_pairs};
    penetration_as_point_pair::Callback(object_A_ptr, object_B_ptr,
                                        &point_data);
  }
  // Tell the broadphase to keep searching.
  return false;
}

}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
