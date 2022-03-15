#pragma once

#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/deformable_contact_geometries.h"
#include "drake/geometry/proximity/deformable_contact_surface.h"
#include "drake/geometry/proximity/deformable_rigid_contact_pair.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/query_results/deformable_contact_data.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {

/* This class stores all instantiated representations of declared geometries for
 deformabe contact. They are keyed by the geometry's global GeometryId.

 In order for a geometry with id `g_id` to have a representation in this
 collection, it must:

   - be declared via calling MaybeAddGeometry();
   - the infrastructure must support the type of representation for that
     particular geometry type;
   - it must have a valid set of properties to fully specify the representation;
   - the geometry cannot have been subsequently removed via a call to
     RemoveGeometry().

 If two geometries are in contact, in order to produce the corresponding
 ContactSurface, both ids must have a valid representation in this set.  */
class Geometries final : public ShapeReifier {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Geometries);

  Geometries() = default;

  /* Returns the representation of the deformable geometry with the given `id`.
   */
  const DeformableGeometry& deformable_geometry(GeometryId id) const {
    if (is_deformable(id)) return deformable_geometries_.at(id);
    throw std::runtime_error(
        fmt::format("There is no deformable geometry with GeometryId {}", id));
  }

  /* Returns the representation of the rigid geometry with the given `id`. */
  const RigidGeometry& rigid_geometry(GeometryId id) const {
    if (is_rigid(id)) return rigid_geometries_.at(id);
    throw std::runtime_error(
        fmt::format("There is no rigid geometry with GeometryId {}", id));
  }

  bool is_rigid(GeometryId id) const {
    return rigid_geometries_.count(id) != 0;
  }

  bool is_deformable(GeometryId id) const {
    return deformable_geometries_.count(id) != 0;
  }

  int num_deformable_geometry() const { return deformable_geometries_.size(); }

  /* Removes the geometry (if it has a deformable contact representation).  */
  void RemoveGeometry(GeometryId id);

  /* Examines the given shape and properties, adding a deformable contact
   representation as indicated by the `properties` and supported by the current
   infrastructure. No exception is thrown if the given shape is not supported in
   the current infrastructure. However, if it *is* supported, but the properties
   are malformed, an exception will be thrown.

   @param shape         The shape to possibly represent.
   @param id            The unique identifier for the geometry.
   @param properties    The proximity properties which will determine if a
                        hydroelastic representation is requested.
   @throws std::exception if the shape is a supported type but the properties
                          are malformed.
   @pre There is no previous representation associated with id.  */
  void MaybeAddGeometry(const Shape& shape, GeometryId id,
                        const ProximityProperties& properties);

  /* Updates the world pose of the rigid geometry with the given id, if it
   exists, to `X_WG`. */
  void UpdateRigidWorldPose(GeometryId id,
                            const math::RigidTransform<double>& X_WG) {
    if (is_rigid(id)) {
      rigid_geometries_.at(id).set_pose_in_world(X_WG);
    }
  }

  /* If the deformable geometry with `id` exists, updates the vertex positions
   of the geometry (in the mesh frame M) to `q_MG`. */
  void UpdateDeformableVertexPositions(
      GeometryId id, const Eigen::Ref<const VectorX<double>>& q_MG) {
    if (is_deformable(id)) {
      deformable_geometries_.at(id).UpdateVertexPositions(q_MG);
    }
  }

  /* For all registered deformable bodies, computes the contact data of that
   deformable body with all registered rigid bodies. Assumes the vertex
   positions and poses of all registered deformable and rigid bodies are up to
   date. */
  void ComputeDeformableContactData(std::vector<DeformableContactData<double>>*
                                        deformable_contact_data) const {
    deformable_contact_data->clear();
    deformable_contact_data->reserve(num_deformable_geometry());
    for (const auto& it : deformable_geometries_) {
      deformable_contact_data->emplace_back(
          CalcDeformableContactData(it.first));
    }
  }

 private:
  // Data to be used during reification. It is passed as the `user_data`
  // parameter in the ImplementGeometry API.
  struct ReifyData {
    bool is_rigid{false};
    GeometryId id;
    const ProximityProperties& properties;
  };

  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Sphere& sphere, void* user_data) override;
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) override;
  void ImplementGeometry(const HalfSpace&, void* user_data) override;
  void ImplementGeometry(const Box& box, void* user_data) override;
  void ImplementGeometry(const Capsule& capsule, void* user_data) override;
  void ImplementGeometry(const Ellipsoid& ellipsoid, void* user_data) override;
  void ImplementGeometry(const Mesh&, void*) override;
  void ImplementGeometry(const Convex& convex, void* user_data) override;

  template <typename ShapeType>
  void MakeShape(const ShapeType& shape, const ReifyData& data);

  /* Calculates the contact data for the deforamble body with the given id.
   @pre A deformable geometry with `deformable_id` exists. */
  DeformableContactData<double> CalcDeformableContactData(
      GeometryId deformable_id) const {
    DRAKE_DEMAND(is_deformable(deformable_id));
    std::vector<internal::deformable::DeformableRigidContactPair<double>>
        deformable_rigid_contact_pairs;
    for (const auto& it : rigid_geometries_) {
      DeformableRigidContactPair<double> contact_pair =
          CalcDeformableRigidContactPair(it.first, deformable_id);
      if (contact_pair.num_contact_points() != 0) {
        deformable_rigid_contact_pairs.emplace_back(std::move(contact_pair));
      }
    }
    return {std::move(deformable_rigid_contact_pairs),
            deformable_geometries_[deformable_id]};
  }

  DeformableRigidContactPair<double> CalcDeformableRigidContactPair(
      GeometryId rigid_id, GeometryId deformable_id) const {
    DRAKE_DEMAND(is_deformable(deforamble_id));
    DRAKE_DEMAND(is_rigid(rigid_id));

    const DeformableVolumeMesh<double>& deformable_tet_mesh =
        deformable_geometries_[deformable_id].deformable_volume_mesh();
    const RigidGeometry& rigid_geometry = rigid_geometries_[rigid_id];
    const math::RigidTransform<double>& X_WR = rigid_geometry.pose_in_world();
    const auto& rigid_bvh = rigid_geometry.rigid_mesh().bvh();
    const auto& rigid_tri_mesh = rigid_geometry.rigid_mesh().mesh();

    DeformableContactSurface<double> contact_surface =
        ComputeTetMeshTriMeshContact(deformable_tet_mesh, rigid_tri_mesh,
                                     rigid_bvh, X_WR);
    return internal::DeformableRigidContactPair<double>(
        std::move(contact_surface), rigid_id, deformable_id);
  }

  // The representations of all deformable geometries.
  std::unordered_map<GeometryId, DeformableGeometry> deformable_geometries_;

  // The representations of all rigid geometries.
  std::unordered_map<GeometryId, RigidGeometry> rigid_geometries_;
};

}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
