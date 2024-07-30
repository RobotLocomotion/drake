#pragma once

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <fmt/format.h>

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/collision_filter.h"
#include "drake/geometry/proximity/field_intersection.h"
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
namespace hydroelastic DRAKE_NO_EXPORT {

/* Supporting data for the shape-to-shape hydroelastic contact callback (see
 Callback below). It includes:

    - A collision filter instance.
    - The T-valued poses of _all_ geometries in the corresponding SceneGraph,
      each indexed by its corresponding geometry's GeometryId.
    - The representation of all geometries that have been prepped for computing
      contact surfaces.
    - The choice of how to represent contact polygons.
    - A vector of contact surfaces -- one instance of ContactSurface for
      every supported, unfiltered penetrating pair.

 @tparam T The computation scalar.  */
template <typename T>
struct CallbackData {
  /* Constructs the fully-specified callback data. The values are as described
   in the class documentation. All parameters are aliased in the data and must
   remain valid at least as long as the CallbackData instance.

   @param collision_filter_in     The collision filter system. Aliased.
   @param X_WGs_in                The T-valued poses. Aliased.
   @param geometries_in           The set of all hydroelastic geometric
                                  representations. Aliased.
   @param representation          Controls the mesh representation of
                                  the contact surface. See
                                  @ref contact_surface_discrete_representation
                                  "contact surface representation" for more
                                  details.
   @param surfaces_in             The output results. Aliased.  */
  CallbackData(
      const CollisionFilter* collision_filter_in,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>* X_WGs_in,
      const Geometries* geometries_in,
      HydroelasticContactRepresentation representation_in,
      std::vector<ContactSurface<T>>* surfaces_in)
      : collision_filter(*collision_filter_in),
        X_WGs(*X_WGs_in),
        geometries(*geometries_in),
        representation(representation_in),
        surfaces(*surfaces_in) {
    DRAKE_DEMAND(collision_filter_in != nullptr);
    DRAKE_DEMAND(X_WGs_in != nullptr);
    DRAKE_DEMAND(geometries_in != nullptr);
    DRAKE_DEMAND(surfaces_in != nullptr);
  }

  /* The collision filter system.  */
  const CollisionFilter& collision_filter;

  /* The T-valued poses of all geometries.  */
  const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs;

  /* The hydroelastic geometric representations.  */
  const Geometries& geometries;

  /* The requested mesh representation type. */
  const HydroelasticContactRepresentation representation;

  /* The results of the distance query.  */
  std::vector<ContactSurface<T>>& surfaces;
};

enum class CalcContactSurfaceResult {
  kCalculated,          //< Computation was successful; a contact surface is
                        //< only produced if the objects were in contact.
  kUnsupported,         //< Contact surface can't be computed for the geometry
                        //< pair.
  kHalfSpaceHalfSpace,  //< Contact between two half spaces; not allowed.
  kRigidRigid,          //< Contact between two rigid geometries; not allowed.
  kCompliantHalfSpaceCompliantMesh,  //< Contact between a compliant mesh and a
                                     //< compliant half space; not allowed.
};

/* Computes ContactSurface using the algorithm appropriate to the Shape types
 represented by the given `soft` and `rigid` geometries.
 @pre The geometries are not *both* half spaces.  */
template <typename T>
std::unique_ptr<ContactSurface<T>> DispatchRigidSoftCalculation(
    const SoftGeometry& soft, const math::RigidTransform<T>& X_WS,
    GeometryId id_S, const RigidGeometry& rigid,
    const math::RigidTransform<T>& X_WR, GeometryId id_R,
    HydroelasticContactRepresentation representation) {
  if (soft.is_half_space() || rigid.is_half_space()) {
    if (soft.is_half_space()) {
      DRAKE_DEMAND(!rigid.is_half_space());
      // Soft half space with rigid mesh.
      const TriangleSurfaceMesh<double>& mesh_R = rigid.mesh();
      const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_R = rigid.bvh();
      return ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
          id_S, X_WS, soft.pressure_scale(), id_R, mesh_R, bvh_R, X_WR,
          representation);
    } else {
      // Soft volume vs rigid half space.
      const VolumeMeshFieldLinear<double, double>& field_S =
          soft.pressure_field();
      const Bvh<Obb, VolumeMesh<double>>& bvh_S = soft.bvh();
      return ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(
          id_S, field_S, bvh_S, X_WS, id_R, X_WR, representation);
    }
  } else {
    // soft cannot be a half space; so this must be mesh-mesh.
    const VolumeMeshFieldLinear<double, double>& field_S =
        soft.pressure_field();
    const Bvh<Obb, VolumeMesh<double>>& bvh_S = soft.bvh();
    const TriangleSurfaceMesh<double>& mesh_R = rigid.mesh();
    const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_R = rigid.bvh();

    return ComputeContactSurfaceFromSoftVolumeRigidSurface(
        id_S, field_S, bvh_S, X_WS, id_R, mesh_R, bvh_R, X_WR, representation);
  }
}

/* Computes ContactSurface using the algorithm appropriate to the Shape types
 represented by the given `compliant` geometries.
 @pre None of the geometries are half spaces. */
template <typename T>
std::unique_ptr<ContactSurface<T>> DispatchCompliantCompliantCalculation(
    const SoftGeometry& compliant0_F, const math::RigidTransform<T>& X_WF,
    GeometryId id0, const SoftGeometry& compliant1_G,
    const math::RigidTransform<T>& X_WG, GeometryId id1,
    HydroelasticContactRepresentation representation) {
  DRAKE_DEMAND(!compliant0_F.is_half_space() && !compliant1_G.is_half_space());

  const VolumeMeshFieldLinear<double, double>& field0_F =
      compliant0_F.pressure_field();
  const Bvh<Obb, VolumeMesh<double>>& bvh0_F = compliant0_F.bvh();
  const VolumeMeshFieldLinear<double, double>& field1_G =
      compliant1_G.pressure_field();
  const Bvh<Obb, VolumeMesh<double>>& bvh1_G = compliant1_G.bvh();

  return ComputeContactSurfaceFromCompliantVolumes(
      id0, field0_F, bvh0_F, X_WF, id1, field1_G, bvh1_G, X_WG, representation);
}

#if 0
// This function is slated for demolition.
/* Calculates the contact surface (if it exists) between two potentially
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

  auto [result, surface] =
      MaybeMakeContactSurface(encoding_a.id(), encoding_b.id(), *data);
  if (surface != nullptr) {
    DRAKE_DEMAND(surface->id_M() < surface->id_N());
    data->surfaces.emplace_back(std::move(*surface));
  }
  return result;
}
#endif

template <typename T>
struct MaybeMakeContactSurfaceResult {
  CalcContactSurfaceResult result;
  std::unique_ptr<ContactSurface<T>> surface;
};

/* Makes the contact surface (if it exists) between two potentially
 colliding geometries.

 @param id0         Id of the first object in the pair (order insignificant).
 @param id1         Id of the second object in the pair (order insignificant).
 @param[out] callback_data   Supporting data to compute the contact surface.
 @returns a struct containing both the result code, and the new surface, if any.
 @tparam T  The scalar type for the query.  */
template <typename T>
MaybeMakeContactSurfaceResult<T> MaybeMakeContactSurface(
    GeometryId id0, GeometryId id1, const CallbackData<T>& data) {
  // One or two objects have vanished. We can report that we're done
  // calculating the contact (no contact).
  if (data.geometries.is_vanished(id0) || data.geometries.is_vanished(id1)) {
    return {CalcContactSurfaceResult::kCalculated, nullptr};
  }

  const HydroelasticType type_A = data.geometries.hydroelastic_type(id0);
  const HydroelasticType type_B = data.geometries.hydroelastic_type(id1);

  // One or two objects have no hydroelastic type.
  if (type_A == HydroelasticType::kUndefined ||
      type_B == HydroelasticType::kUndefined) {
    return {CalcContactSurfaceResult::kUnsupported, nullptr};
  }

  // Rigid-rigid contact is not supported in hydroelastic contact model.
  // Callers might optionally fall back to point contact model.
  if (type_A == HydroelasticType::kRigid &&
      type_B == HydroelasticType::kRigid) {
    return {CalcContactSurfaceResult::kRigidRigid, nullptr};
  }

  // Compliant-compliant contact.
  if (type_A == HydroelasticType::kSoft && type_B == HydroelasticType::kSoft) {
    // Enforce consistent ordering for reproducibility/repeatability of
    // simulation since the same pair of geometries (A,B) may be called
    // either as (A,B) or (B,A).
    if (id0.get_value() > id1.get_value()) {
      std::swap(id0, id1);
    }
    const SoftGeometry& soft0 = data.geometries.soft_geometry(id0);
    const SoftGeometry& soft1 = data.geometries.soft_geometry(id1);

    // Halfspace vs. halfspace is not supported.
    if (soft0.is_half_space() && soft1.is_half_space()) {
      return {CalcContactSurfaceResult::kHalfSpaceHalfSpace, nullptr};
    }

    // Compliant-halfspace vs. compliant-mesh is not supported.
    if (soft0.is_half_space() || soft1.is_half_space()) {
      return {CalcContactSurfaceResult::kCompliantHalfSpaceCompliantMesh,
              nullptr};
    }

    // Compliant mesh vs. compliant mesh.
    DRAKE_DEMAND(!soft0.is_half_space() && !soft1.is_half_space());
    std::unique_ptr<ContactSurface<T>> surface =
        DispatchCompliantCompliantCalculation(soft0, data.X_WGs.at(id0), id0,
                                              soft1, data.X_WGs.at(id1), id1,
                                              data.representation);
    return {CalcContactSurfaceResult::kCalculated, std::move(surface)};
  }

  // Rigid-compliant contact
  DRAKE_DEMAND((type_A == HydroelasticType::kRigid &&
                type_B == HydroelasticType::kSoft) ||
               (type_A == HydroelasticType::kSoft &&
                type_B == HydroelasticType::kRigid));

  bool A_is_rigid = type_A == HydroelasticType::kRigid;
  const GeometryId id_S = A_is_rigid ? id1 : id0;
  const GeometryId id_R = A_is_rigid ? id0 : id1;

  const SoftGeometry& soft = data.geometries.soft_geometry(id_S);
  const RigidGeometry& rigid = data.geometries.rigid_geometry(id_R);

  if (soft.is_half_space() && rigid.is_half_space()) {
    return {CalcContactSurfaceResult::kHalfSpaceHalfSpace, nullptr};
  }

  const math::RigidTransform<T>& X_WS(data.X_WGs.at(id_S));
  const math::RigidTransform<T>& X_WR(data.X_WGs.at(id_R));

  std::unique_ptr<ContactSurface<T>> surface = DispatchRigidSoftCalculation(
      soft, X_WS, id_S, rigid, X_WR, id_R, data.representation);

  return {CalcContactSurfaceResult::kCalculated, std::move(surface)};
}

#if 0
// This function is slated for demolition.
/* Assess contact between two objects -- if it can't be determined with
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

  const bool can_collide =
      data.collision_filter.CanCollideWith(encoding_a.id(), encoding_b.id());

  if (can_collide) {
    CalcContactSurfaceResult result =
        MaybeCalcContactSurface(object_A_ptr, object_B_ptr, &data);

    // Surface calculated; we're done.
    if (result == CalcContactSurfaceResult::kCalculated) return false;

    RejectContactSurfaceResult(result, object_A_ptr, object_B_ptr, data);
  }

  // Tell the broadphase to keep searching.
  return false;
}
#endif

/* @throws a std::exception with an appropriate error message for the various
 result codes that indicate failure.
 @pre result != kCalculated
*/
template <typename T>
[[noreturn]] void RejectContactSurfaceResult(
    CalcContactSurfaceResult result, fcl::CollisionObjectd* object_A_ptr,
    fcl::CollisionObjectd* object_B_ptr, const CallbackData<T>& data) {
  // Give a slightly better diagnostic for a misplaced happy result code.
  DRAKE_DEMAND(result != CalcContactSurfaceResult::kCalculated);
  const EncodedData encoding_a(*object_A_ptr);
  const EncodedData encoding_b(*object_B_ptr);

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
    case CalcContactSurfaceResult::kRigidRigid:
      throw std::logic_error(fmt::format(
          "Requested contact between two rigid objects ({} with id "
          "{}, {} with id {}); that is not allowed in hydroelastic-only "
          "contact. Please consider using hydroelastics with point-contact "
          "fallback, e.g., QueryObject::ComputeContactSurfacesWithFallback() "
          "or MultibodyPlant::set_contact_model("
          "ContactModel::kHydroelasticWithFallback)",
          GetGeometryName(*object_A_ptr), encoding_a.id(),
          GetGeometryName(*object_B_ptr), encoding_b.id()));
    case CalcContactSurfaceResult::kCompliantHalfSpaceCompliantMesh:
      throw std::logic_error(fmt::format(
          "Requested hydroelastic contact between two compliant geometries, "
          "one of which is a half space ({} with id {}, {} with id {}); "
          "that is not allowed",
          GetGeometryName(*object_A_ptr), encoding_a.id(),
          GetGeometryName(*object_B_ptr), encoding_b.id()));
    case CalcContactSurfaceResult::kHalfSpaceHalfSpace:
      throw std::logic_error(fmt::format(
          "Requested contact between two half spaces with ids {} and {}; "
          "that is not allowed",
          encoding_a.id(), encoding_b.id()));
    case CalcContactSurfaceResult::kCalculated:
      // This should never happen (see DRAKE_DEMAND()) above), but is here for
      // compiler switch code completeness checking.
      break;
  }
  DRAKE_UNREACHABLE();
}

/* Supporting data for the shape-to-shape hydroelastic contact callback with
 fallback (see CallbackWithFallback below). It includes:

    - CallbackData for strict hydroelastic contact (see above).
    - A vector of contacts represented as point pairs -- comprising of those
      contacts which could not be evaluated with hydroelastic models.

 @tparam T The computation scalar.  */
template <typename T>
struct CallbackWithFallbackData {
  CallbackData<T> data;
  std::vector<PenetrationAsPointPair<T>>* point_pairs;
};

#if 0
// This function is slated for demolition.
/* Assess contact between two objects -- if it can't be determined with
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
  // DRAKE_DEMAND(false);  // There are some tests.
  auto& data = *static_cast<CallbackWithFallbackData<T>*>(callback_data);

  const EncodedData encoding_a(*object_A_ptr);
  const EncodedData encoding_b(*object_B_ptr);

  const bool can_collide = data.data.collision_filter.CanCollideWith(
      encoding_a.id(), encoding_b.id());

  if (can_collide) {
    CalcContactSurfaceResult result =
        MaybeCalcContactSurface(object_A_ptr, object_B_ptr, &data.data);

    // Surface calculated; we're done.
    if (result == CalcContactSurfaceResult::kCalculated) return false;

    // Fall back to point pair.
    penetration_as_point_pair::CallbackData<T> point_data{
        &data.data.collision_filter, &(data.data.X_WGs), data.point_pairs};
    penetration_as_point_pair::Callback<T>(object_A_ptr, object_B_ptr,
                                           &point_data);
  }
  // Tell the broadphase to keep searching.
  return false;
}
#endif

// clang-format off
}  // namespace hydroelastic
// clang-format on
}  // namespace internal
}  // namespace geometry
}  // namespace drake
