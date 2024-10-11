#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/proximity/collision_filter.h"
#include "drake/geometry/proximity/deformable_contact_geometries.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/query_results/deformable_contact.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {

/* This class stores all instantiated representations of declared geometries for
 deformable contact. They are keyed by the geometry's global GeometryId.

 In order for a geometry with id `g_id` to have a representation in this
 collection, it must:

   - be declared via successful calls to MaybeAddRigidGeometry() (for
     non-deformable geometries) or AddDeformableGeometry() (for deformable
     geometries);
   - the geometry cannot have been subsequently removed via a call to
     RemoveGeometry().

 If a geometry satisfies the requirements above, we say that the geometry has a
 "deformable contact representation". If two geometries are in contact, in order
 to produce the corresponding contact data, both ids must have a valid
 representation in this data structure. Right now, we only support deformable
 vs. rigid contact. In the future, we will also support deformable vs.
 deformable contact. Rigid vs. rigid contact is handled through point contact or
 hydroelastics outside of this class. */
class Geometries final : public ShapeReifier {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Geometries);

  Geometries() = default;
  ~Geometries() final;

  /* Returns true if a rigid (non-deformable) geometry representation with the
   given `id` exists. */
  bool is_rigid(GeometryId id) const {
    return rigid_geometries_.contains(id) ||
           rigid_geometries_pending_.contains(id);
  }

  /* Returns true if a deformable geometry representation with the given `id`
   exists. */
  bool is_deformable(GeometryId id) const {
    return deformable_geometries_.contains(id);
  }

  /* Removes the geometry (if it has a deformable contact representation). No-op
   if no geometry with a deformable contact representation exists with the
   provided id. */
  void RemoveGeometry(GeometryId id);

  // TODO(xuchenhan-tri): Currently we rely on the resolution_hint property to
  // determine whether a rigid geometry participates in deformable contact. This
  // is very much an ad-hoc measure to quickly enable deformable contact of some
  // sort. In the future, we need to decouple the ability of a rigid geometry to
  // participate in *deformable contact* from its *hydroelastic* properties.

  /* Examines the given shape and properties, adding a rigid geometry
   representation if
   1. `props` specifies the resolution hint for the mesh representation of the
      rigid geometry.
   2. The `shape` type is supported for rigid representation for deformable
      contact. The set of supported geometries is the set of all supported hydro
      rigid geometries minus half space. We use the same implementation that the
      hydro-rigid reifier is using for supported shapes.
   This function is a no-op if the resolution_hint property is not specified and
   logs a one-time warning if the shape is not supported for deformable contact.

   @param shape         The shape to possibly represent.
   @param id            The unique identifier for the geometry.
   @param properties    The proximity properties that specifies the properties
                        of the rigid representation.
   @param X_WG          The pose of the geometry in the world frame.
   @throws std::exception if resolution hint <= 0 for the following shapes: Box,
           Sphere, Cylinder, Capsule, and Ellipsoid. Note that Mesh and Convex
           don't restrict the range of resolution_hint.
   @pre There is no previous representation associated with id.  */
  void MaybeAddRigidGeometry(const Shape& shape, GeometryId id,
                             const ProximityProperties& props,
                             const math::RigidTransform<double>& X_WG);

  /* Updates the world pose of the rigid geometry with the given id, if it
   exists, to `X_WG`. */
  void UpdateRigidWorldPose(GeometryId id,
                            const math::RigidTransform<double>& X_WG);

  /* Adds a deformable geometry whose contact mesh representation is given by
   `mesh`.

   @param id     The unique identifier for the geometry.
   @param mesh   The volume mesh representation of the deformable geometry.
   @pre There is no previous representation associated with id. */
  void AddDeformableGeometry(GeometryId id, VolumeMesh<double> mesh);

  /* If a deformable geometry with `id` exists, updates the vertex positions
   of the geometry (in the world frame) to `q_WG`. */
  void UpdateDeformableVertexPositions(
      GeometryId id, const Eigen::Ref<const VectorX<double>>& q_WG);

  /* For each registered deformable geometry, computes the contact data of it
   with respect to all registered rigid geometries and all other deformable
   geometries. Assumes the vertex positions and poses of all registered
   deformable and rigid geometries are up to date.
  */
  DeformableContact<double> ComputeDeformableContact(
      const CollisionFilter& collision_filter) const;

 private:
  friend class GeometriesTester;

  /* Data to be used during reification. It is passed as the `user_data`
   parameter in the ImplementGeometry API. */
  struct ReifyData {
    GeometryId id;
    const ProximityProperties& properties;
  };

  void ImplementGeometry(const Box& box, void* user_data) override;
  void ImplementGeometry(const Capsule& capsule, void* user_data) override;
  void ImplementGeometry(const Convex& convex, void* user_data) override;
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) override;
  void ImplementGeometry(const Ellipsoid& ellipsoid, void* user_data) override;
  void ImplementGeometry(const HalfSpace& half_space, void* user_data) override;
  void ImplementGeometry(const Mesh& mesh, void* user_data) override;
  void ImplementGeometry(const MeshcatCone& cone, void* user_data) override;
  void ImplementGeometry(const Sphere& sphere, void* user_data) override;

  /* Makes a rigid (non-deformable) geometry from a supported shape type using
   the given `data`. Unsupported geometries are silently ignored. */
  template <typename ShapeType>
  void AddRigidGeometry(const ShapeType& shape, const ReifyData& data);

  /* Moves rigid_geometries_pending_ into rigid_geometries_. */
  void FlushPendingRigidGeometry();

  // The representations of all deformable geometries.
  std::unordered_map<GeometryId, DeformableGeometry> deformable_geometries_;

  // The representations of all rigid geometries. For performance, we store them
  // in 'pending' until a deformable geometry appears.
  std::unordered_map<GeometryId, GeometryInstance> rigid_geometries_pending_;
  std::unordered_map<GeometryId, RigidGeometry> rigid_geometries_;

  // This is always true in practice (there is no setter). Only certain unit
  // tests' obscene need to access our private member fields using friendship
  // will ever set it to false, so that they don't need to worry about the two
  // different rigid geometry maps.
  bool enable_rigid_geometries_pending_{true};
};

}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
