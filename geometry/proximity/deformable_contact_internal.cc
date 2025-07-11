#include "drake/geometry/proximity/deformable_contact_internal.h"

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/proximity/deformable_contact_geometries.h"
#include "drake/geometry/proximity/deformable_field_intersection.h"
#include "drake/geometry/proximity/deformable_mesh_intersection.h"
#include "drake/geometry/proximity/hydroelastic_internal.h"

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {

Geometries::~Geometries() = default;

void Geometries::RemoveGeometry(GeometryId id) {
  deformable_geometries_.erase(id);
  rigid_geometries_pending_.erase(id);
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
  if (props.HasProperty(kHydroGroup, kRezHint) &&
      props.HasProperty(kHydroGroup, kElastic)) {
    ReifyData data{id, props};
    shape.Reify(this, &data);
    UpdateRigidWorldPose(id, X_WG);
  }
}

void Geometries::UpdateRigidWorldPose(
    GeometryId id, const math::RigidTransform<double>& X_WG) {
  if (auto iter = rigid_geometries_pending_.find(id);
      iter != rigid_geometries_pending_.end()) {
    iter->second.set_pose(X_WG);
    return;
  }
  if (auto iter = rigid_geometries_.find(id); iter != rigid_geometries_.end()) {
    iter->second.set_pose_in_world(X_WG);
    return;
  }
}

void Geometries::AddDeformableGeometry(
    GeometryId id, VolumeMesh<double> volume_mesh,
    TriangleSurfaceMesh<double> surface_mesh,
    std::vector<int> surface_index_to_volume_index,
    std::vector<int> surface_tri_to_volume_tet) {
  deformable_geometries_.insert(
      {id, DeformableGeometry(std::move(volume_mesh), std::move(surface_mesh),
                              std::move(surface_index_to_volume_index),
                              std::move(surface_tri_to_volume_tet))});
  FlushPendingRigidGeometry();
}

void Geometries::UpdateDeformableVertexPositions(
    GeometryId id, const Eigen::Ref<const VectorX<double>>& q_WV,
    const Eigen::Ref<const VectorX<double>>& q_WS) {
  if (is_deformable(id)) {
    deformable_geometries_.at(id).UpdateVertexPositions(q_WV, q_WS);
  }
}

DeformableContact<double> Geometries::ComputeDeformableContact(
    const CollisionFilter& collision_filter) const {
  DeformableContact<double> result;
  // Set up a temporary cache of pointers to signed distance fields, so
  // we can compute the fields only once per deformable geometry. We don't
  // want to compute them again for each of the O(n^2) pairs of
  // deformable-deformable contacts of n deformable geometries.
  //
  // N.B. It is valid as long as deformable geometries do not get
  // UpdateVertexPositions().
  //
  // N.B. It is valid as long as we do not call CalcSignedDistanceField()
  // again. When we call CalcSignedDistanceField(), it creates a new
  // VolumeMeshFieldLinear instance for the signed distance field.
  std::unordered_map<GeometryId, const VolumeMeshFieldLinear<double, double>*>
      signed_distance_fields;
  for (const auto& [deformable_id, deformable_geometry] :
       deformable_geometries_) {
    signed_distance_fields[deformable_id] =
        &deformable_geometry.CalcSignedDistanceField();
    result.RegisterDeformableGeometry(
        deformable_id,
        deformable_geometry.deformable_volume().mesh().num_vertices());
  }

  // Add deformable-rigid contacts.
  for (const auto& [deformable_id, deformable_geometry] :
       deformable_geometries_) {
    DRAKE_ASSERT(collision_filter.HasGeometry(deformable_id));
    for (const auto& [rigid_id, rigid_geometry] : rigid_geometries_) {
      DRAKE_ASSERT(collision_filter.HasGeometry(rigid_id));
      if (collision_filter.CanCollideWith(deformable_id, rigid_id)) {
        const math::RigidTransform<double>& X_WR =
            rigid_geometry.pose_in_world();
        const auto& rigid_bvh = rigid_geometry.mesh().bvh();
        const auto& pressure_field_R = rigid_geometry.mesh().pressure();
        // Deformable geometry is in the world frame.
        const auto X_RD = X_WR.inverse();
        AddDeformableRigidContactSurface(
            deformable_geometry.deformable_surface(),
            deformable_geometry.deformable_volume(),
            deformable_geometry.surface_index_to_volume_index(),
            deformable_geometry.surface_tri_to_volume_tet(), deformable_id,
            rigid_id, pressure_field_R, rigid_bvh, X_RD, &result);
      }
    }
  }
  // Add deformable-deformable contacts.
  // This double-loop deformable-deformable contact is more complicated
  // than the one for deformable-rigid contact because we want only the
  // pairs (deformable_i, deformable_j) without
  // (deformable_j, deformable_i) or (deformable_i, deformable_i).
  // Notice that we use the container deformable_geometries_ as read-only
  // since this method is const.
  for (std::unordered_map<GeometryId, DeformableGeometry>::const_iterator it0 =
           deformable_geometries_.begin();
       it0 != deformable_geometries_.end(); ++it0) {
    const GeometryId deformable0_id = it0->first;
    DRAKE_ASSERT(collision_filter.HasGeometry(deformable0_id));
    // Notice that we do not change the container deformable_geometries_, so
    // it's safe to use std::next() in this nested loop. The std::next()
    // requires LegacyInputIterator which is guaranteed multipass-safe.
    for (std::unordered_map<GeometryId, DeformableGeometry>::const_iterator
             it1 = std::next(it0);
         it1 != deformable_geometries_.end(); ++it1) {
      const GeometryId deformable1_id = it1->first;
      DRAKE_ASSERT(collision_filter.HasGeometry(deformable1_id));
      if (collision_filter.CanCollideWith(deformable0_id, deformable1_id)) {
        if (deformable1_id < deformable0_id) {
          AddDeformableDeformableContactSurface(
              *signed_distance_fields.at(deformable0_id),
              it0->second.deformable_volume(), deformable0_id,
              *signed_distance_fields.at(deformable1_id),
              it1->second.deformable_volume(), deformable1_id, &result);
        } else {
          AddDeformableDeformableContactSurface(
              *signed_distance_fields.at(deformable1_id),
              it1->second.deformable_volume(), deformable1_id,
              *signed_distance_fields.at(deformable0_id),
              it0->second.deformable_volume(), deformable0_id, &result);
        }
      }
    }
  }

  return result;
}

const Aabb& Geometries::GetDeformableAabbInWorld(GeometryId geometry_id) const {
  DRAKE_DEMAND(is_deformable(geometry_id));
  return deformable_geometries_.at(geometry_id).aabb_in_world();
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
  if (enable_rigid_geometries_pending_ && deformable_geometries_.empty()) {
    GeometryInstance instance(math::RigidTransform<double>{}, shape, "pending");
    instance.set_proximity_properties(data.properties);
    rigid_geometries_pending_.insert({data.id, std::move(instance)});
    return;
  }
  std::optional<RigidGeometry> rigid_geometry =
      internal::deformable::MakeMeshRepresentation(shape, data.properties);
  if (rigid_geometry) {
    rigid_geometries_.insert({data.id, std::move(*rigid_geometry)});
  }
}

void Geometries::FlushPendingRigidGeometry() {
  DRAKE_DEMAND(!deformable_geometries_.empty());
  auto worklist = std::move(rigid_geometries_pending_);
  rigid_geometries_pending_.clear();
  for (const auto& [id, geometry_instance] : worklist) {
    const ProximityProperties* props = geometry_instance.proximity_properties();
    DRAKE_DEMAND(props != nullptr);
    MaybeAddRigidGeometry(geometry_instance.shape(), id, *props,
                          geometry_instance.pose());
  }
}

}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
