#include "drake/geometry/proximity/hydroelastic_calculator.h"

#include <utility>

#include <fmt/format.h>

#include "drake/geometry/proximity/field_intersection.h"
#include "drake/geometry/proximity/mesh_half_space_intersection.h"
#include "drake/geometry/proximity/mesh_intersection.h"
#include "drake/geometry/proximity/mesh_plane_intersection.h"
#include "drake/geometry/proximity/proximity_utilities.h"

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

template <typename T>
typename ContactCalculator<T>::MaybeMakeContactSurfaceResult
ContactCalculator<T>::MaybeMakeContactSurface(GeometryId id_A,
                                              GeometryId id_B) const {
  // One or two objects have vanished. We can report that we're done
  // calculating the contact (no contact).
  if (geometries_.is_vanished(id_A) || geometries_.is_vanished(id_B)) {
    return {ContactSurfaceResult::kCalculated, nullptr};
  }

  const HydroelasticType type_A = geometries_.hydroelastic_type(id_A);
  const HydroelasticType type_B = geometries_.hydroelastic_type(id_B);

  // One or two objects have no hydroelastic type.
  if (type_A == HydroelasticType::kUndefined ||
      type_B == HydroelasticType::kUndefined) {
    return {ContactSurfaceResult::kUnsupported, nullptr};
  }

  // Rigid-rigid contact is not supported in hydroelastic contact model.
  // Callers might optionally fall back to point contact model.
  if (type_A == HydroelasticType::kRigid &&
      type_B == HydroelasticType::kRigid) {
    return {ContactSurfaceResult::kRigidRigid, nullptr};
  }

  // Compliant-compliant contact.
  if (type_A == HydroelasticType::kSoft && type_B == HydroelasticType::kSoft) {
    // Enforce consistent ordering for reproducibility/repeatability of
    // simulation since the same pair of geometries (A,B) may be called
    // either as (A,B) or (B,A).
    if (id_A.get_value() > id_B.get_value()) {
      std::swap(id_A, id_B);
    }
    const SoftGeometry& soft0 = geometries_.soft_geometry(id_A);
    const SoftGeometry& soft1 = geometries_.soft_geometry(id_B);

    // Halfspace vs. halfspace is not supported.
    if (soft0.is_half_space() && soft1.is_half_space()) {
      return {ContactSurfaceResult::kHalfSpaceHalfSpace, nullptr};
    }

    // Compliant-halfspace vs. compliant-mesh is not supported.
    if (soft0.is_half_space() || soft1.is_half_space()) {
      return {ContactSurfaceResult::kCompliantHalfSpaceCompliantMesh, nullptr};
    }

    // Compliant mesh vs. compliant mesh.
    DRAKE_DEMAND(!soft0.is_half_space() && !soft1.is_half_space());
    std::unique_ptr<ContactSurface<T>> surface =
        DispatchCompliantCompliantCalculation(soft0, X_WGs_.at(id_A), id_A,
                                              soft1, X_WGs_.at(id_B), id_B,
                                              representation_);
    return {ContactSurfaceResult::kCalculated, std::move(surface)};
  }

  // Rigid-compliant contact
  DRAKE_DEMAND((type_A == HydroelasticType::kRigid &&
                type_B == HydroelasticType::kSoft) ||
               (type_A == HydroelasticType::kSoft &&
                type_B == HydroelasticType::kRigid));

  bool A_is_rigid = type_A == HydroelasticType::kRigid;
  const GeometryId id_S = A_is_rigid ? id_B : id_A;
  const GeometryId id_R = A_is_rigid ? id_A : id_B;

  const SoftGeometry& soft = geometries_.soft_geometry(id_S);
  const RigidGeometry& rigid = geometries_.rigid_geometry(id_R);

  if (soft.is_half_space() && rigid.is_half_space()) {
    return {ContactSurfaceResult::kHalfSpaceHalfSpace, nullptr};
  }

  const math::RigidTransform<T>& X_WS(X_WGs_.at(id_S));
  const math::RigidTransform<T>& X_WR(X_WGs_.at(id_R));

  std::unique_ptr<ContactSurface<T>> surface = DispatchRigidSoftCalculation(
      soft, X_WS, id_S, rigid, X_WR, id_R, representation_);

  return {ContactSurfaceResult::kCalculated, std::move(surface)};
}

template <typename T>
[[noreturn]] void ContactCalculator<T>::RejectResult(
    ContactSurfaceResult result, fcl::CollisionObjectd* object_A_ptr,
    fcl::CollisionObjectd* object_B_ptr) const {
  // Give a slightly better diagnostic for a misplaced happy result code.
  DRAKE_DEMAND(result != ContactSurfaceResult::kCalculated);
  const EncodedData encoding_a(*object_A_ptr);
  const EncodedData encoding_b(*object_B_ptr);

  const HydroelasticType type_A =
      geometries_.hydroelastic_type(encoding_a.id());
  const HydroelasticType type_B =
      geometries_.hydroelastic_type(encoding_b.id());

  switch (result) {
    case ContactSurfaceResult::kUnsupported:
      throw std::logic_error(fmt::format(
          "Requested a contact surface between a pair of geometries without "
          "hydroelastic representation for at least one shape: a {} {} with "
          "id {} and a {} {} with id {}",
          type_A, GetGeometryName(*object_A_ptr), encoding_a.id(), type_B,
          GetGeometryName(*object_B_ptr), encoding_b.id()));
    case ContactSurfaceResult::kRigidRigid:
      throw std::logic_error(fmt::format(
          "Requested contact between two rigid objects ({} with id "
          "{}, {} with id {}); that is not allowed in hydroelastic-only "
          "contact. Please consider using hydroelastics with point-contact "
          "fallback, e.g., QueryObject::ComputeContactSurfacesWithFallback() "
          "or MultibodyPlant::set_contact_model("
          "ContactModel::kHydroelasticWithFallback)",
          GetGeometryName(*object_A_ptr), encoding_a.id(),
          GetGeometryName(*object_B_ptr), encoding_b.id()));
    case ContactSurfaceResult::kCompliantHalfSpaceCompliantMesh:
      throw std::logic_error(fmt::format(
          "Requested hydroelastic contact between two compliant geometries, "
          "one of which is a half space ({} with id {}, {} with id {}); "
          "that is not allowed",
          GetGeometryName(*object_A_ptr), encoding_a.id(),
          GetGeometryName(*object_B_ptr), encoding_b.id()));
    case ContactSurfaceResult::kHalfSpaceHalfSpace:
      throw std::logic_error(fmt::format(
          "Requested contact between two half spaces with ids {} and {}; "
          "that is not allowed",
          encoding_a.id(), encoding_b.id()));
    case ContactSurfaceResult::kCalculated:
      // This should never happen (see DRAKE_DEMAND()) above), but is here for
      // compiler switch code completeness checking.
      break;
  }
  DRAKE_UNREACHABLE();
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ContactCalculator);

}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
