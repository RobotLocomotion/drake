#include "drake/geometry/proximity/hydroelastic_calculator.h"

#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/geometry/proximity/field_intersection.h"
#include "drake/geometry/proximity/mesh_half_space_intersection.h"
#include "drake/geometry/proximity/mesh_intersection.h"
#include "drake/geometry/proximity/mesh_plane_intersection.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {

template <typename T>
std::unique_ptr<ContactSurface<T>> CalcRigidCompliant(
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
          representation, soft.half_space_margin());
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

template <typename T>
std::unique_ptr<ContactSurface<T>> CalcCompliantCompliant(
    const SoftGeometry& compliant_F, const math::RigidTransform<T>& X_WF,
    GeometryId id_F, const SoftGeometry& compliant_G,
    const math::RigidTransform<T>& X_WG, GeometryId id_G,
    HydroelasticContactRepresentation representation) {
  DRAKE_DEMAND(!compliant_F.is_half_space() && !compliant_G.is_half_space());

  return ComputeContactSurfaceFromCompliantVolumes(
      id_F, compliant_F.soft_mesh(), X_WF, id_G, compliant_G.soft_mesh(), X_WG,
      representation);
}

template <typename T>
ContactCalculator<T>::ContactCalculator(
    const std::unordered_map<GeometryId, math::RigidTransform<T>>* X_WGs,
    const Geometries* geometries,
    HydroelasticContactRepresentation representation)
    : X_WGs_(DRAKE_DEREF(X_WGs)),
      geometries_(DRAKE_DEREF(geometries)),
      representation_(representation) {}

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
    const SoftGeometry& soft_A = geometries_.soft_geometry(id_A);
    const SoftGeometry& soft_B = geometries_.soft_geometry(id_B);

    // Halfspace vs. halfspace is not supported.
    if (soft_A.is_half_space() && soft_B.is_half_space()) {
      return {ContactSurfaceResult::kHalfSpaceHalfSpace, nullptr};
    }

    // Compliant-halfspace vs. compliant-mesh is not supported.
    if (soft_A.is_half_space() || soft_B.is_half_space()) {
      return {ContactSurfaceResult::kCompliantHalfSpaceCompliantMesh, nullptr};
    }

    // Compliant mesh vs. compliant mesh.
    std::unique_ptr<ContactSurface<T>> surface =
        CalcCompliantCompliant(soft_A, X_WGs_.at(id_A), id_A, soft_B,
                               X_WGs_.at(id_B), id_B, representation_);
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

  const math::RigidTransform<T>& X_WS = X_WGs_.at(id_S);
  const math::RigidTransform<T>& X_WR = X_WGs_.at(id_R);

  std::unique_ptr<ContactSurface<T>> surface =
      CalcRigidCompliant(soft, X_WS, id_S, rigid, X_WR, id_R, representation_);

  return {ContactSurfaceResult::kCalculated, std::move(surface)};
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&CalcRigidCompliant<T>, &CalcCompliantCompliant<T>));

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ContactCalculator);

}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
