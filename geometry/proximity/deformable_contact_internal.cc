#include "drake/geometry/proximity/deformable_contact_internal.h"

#include <algorithm>
#include <memory>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/geometry/proximity/deformable_contact_geometries.h"
#include "drake/geometry/proximity/deformable_mesh_intersection.h"
#include "drake/geometry/proximity/hydroelastic_internal.h"

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {

void Geometries::RemoveGeometry(GeometryId id) {
  deformable_geometries_.erase(id);
  rigid_geometries_.erase(id);
}

void Geometries::MaybeAddRigidGeometry(
    const Shape& shape, GeometryId id, const ProximityProperties& props,
    const math::RigidTransform<double>& X_WG) {
  // TODO(xuchenhan-tri): Right now, rigid geometries participating in
  // deformable contact share the property "kRezHint" with hydroelastics. It's
  // reasonable to use the contact mesh with the same resolution for both hydro
  // and deformable contact. Consider reorganizing the proximity properties to
  // make this sharing more explicit. We should also avoid having two copies of
  // the same rigid geometry for both hydro and deformable contact.
  if (props.HasProperty(kHydroGroup, kRezHint)) {
    ReifyData data{id, props};
    shape.Reify(this, &data);
    UpdateRigidWorldPose(id, X_WG);
  }
}

void Geometries::UpdateRigidWorldPose(
    GeometryId id, const math::RigidTransform<double>& X_WG) {
  if (is_rigid(id)) {
    rigid_geometries_.at(id).set_pose_in_world(X_WG);
  }
}

void Geometries::AddDeformableGeometry(GeometryId id, VolumeMesh<double> mesh) {
  deformable_geometries_.insert({id, DeformableGeometry(std::move(mesh))});
}

void Geometries::UpdateDeformableVertexPositions(
    GeometryId id, const Eigen::Ref<const VectorX<double>>& q_WG) {
  if (is_deformable(id)) {
    deformable_geometries_.at(id).UpdateVertexPositions(q_WG);
  }
}

DeformableContact<double> Geometries::ComputeDeformableContact(
    const CollisionFilter& collision_filter) const {
  DeformableContact<double> result;
  for (const auto& [deformable_id, deformable_geometry] :
       deformable_geometries_) {
    const VolumeMeshFieldLinear<double, double>& deformable_sdf =
        deformable_geometry.CalcSignedDistanceField();
    const DeformableVolumeMesh<double>& deformable_mesh =
        deformable_geometry.deformable_mesh();
    result.RegisterDeformableGeometry(deformable_id,
                                      deformable_mesh.mesh().num_vertices());
    DRAKE_ASSERT(collision_filter.HasGeometry(deformable_id));
    for (const auto& [rigid_id, rigid_geometry] : rigid_geometries_) {
      DRAKE_ASSERT(collision_filter.HasGeometry(rigid_id));
      if (collision_filter.CanCollideWith(deformable_id, rigid_id)) {
        const math::RigidTransform<double>& X_WR =
            rigid_geometry.pose_in_world();
        const auto& rigid_bvh = rigid_geometry.rigid_mesh().bvh();
        const auto& rigid_tri_mesh = rigid_geometry.rigid_mesh().mesh();
        AddDeformableRigidContactSurface(
            deformable_sdf, deformable_mesh, deformable_id, rigid_id,
            rigid_tri_mesh, rigid_bvh, X_WR, &result);
      }
    }
  }
  return result;
}

void Geometries::ImplementGeometry(const Box& box, void* user_data) {
  AddRigidGeometry(box, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Capsule& capsule, void* user_data) {
  AddRigidGeometry(capsule, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Convex& convex, void* user_data) {
  AddRigidGeometry(convex, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Cylinder& cylinder, void* user_data) {
  AddRigidGeometry(cylinder, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Ellipsoid& ellipsoid,
                                   void* user_data) {
  AddRigidGeometry(ellipsoid, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const HalfSpace&, void*) {
  static const logging::Warn log_once(
      "Rigid (non-deformable) half spaces are not currently supported for "
      "deformable contact; registration is allowed, but no contact data will "
      "be reported.");
}

void Geometries::ImplementGeometry(const Mesh& mesh, void* user_data) {
  AddRigidGeometry(mesh, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const MeshcatCone&, void*) {
  static const logging::Warn log_once(
      "Rigid (non-deformable) Meshcat cones are not currently supported for "
      "deformable contact; registration is allowed, but no contact data will "
      "be reported.");
}

void Geometries::ImplementGeometry(const Sphere& sphere, void* user_data) {
  AddRigidGeometry(sphere, *static_cast<ReifyData*>(user_data));
}

template <typename ShapeType>
void Geometries::AddRigidGeometry(const ShapeType& shape,
                                  const ReifyData& data) {
  /* Forward to hydroelastics to construct the geometry. */
  std::optional<internal::hydroelastic::RigidGeometry> hydro_rigid_geometry =
      internal::hydroelastic::MakeRigidRepresentation(shape, data.properties);
  /* Unsupported geometries will be handle through the
   `ThrowUnsupportedGeometry()` code path. */
  DRAKE_DEMAND(hydro_rigid_geometry.has_value());
  rigid_geometries_.insert(
      {data.id, RigidGeometry(hydro_rigid_geometry->release_mesh())});
}

}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
