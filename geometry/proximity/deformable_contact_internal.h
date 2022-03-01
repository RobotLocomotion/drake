#pragma once

#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/deformable_volume_mesh.h"
#include "drake/geometry/proximity/hydroelastic_internal.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {

/* Definition of a deformable geometry for contact implementations. To be a
 deformable geometry, a shape must be associated with both:
   - a deformable volume mesh, and
   - an approximate signed distance field in the interior of the mesh. */
class DeformableGeometry {
 public:
  /* Constructs a deformable geometry. */
  DeformableGeometry(
      std::unique_ptr<internal::DeformableVolumeMesh<double>> mesh,
      std::unique_ptr<VolumeMeshFieldLinear<double, double>> signed_distance)
      : mesh_(std::move(mesh)),
        reference_mesh_(std::make_unique<VolumeMesh<double>>(mesh_->mesh())),
        signed_distance_(std::move(signed_distance)) {}

  /* Custom copy assign and construct. */
  DeformableGeometry& operator=(const DeformableGeometry& s);
  DeformableGeometry(const DeformableGeometry& s);
  /* Default move assign and construct. */
  DeformableGeometry(DeformableGeometry&&) = default;
  DeformableGeometry& operator=(DeformableGeometry&&) = default;

  /* Returns the volume mesh representation of the deformable geometry at
   current configuration. */
  const internal::DeformableVolumeMesh<double>& deformable_volume_mesh() const {
    return *mesh_;
  }

  /* Updates the vertex positions of the underlying deformable mesh.
  @param q  A vector of 3N values (where this mesh has N vertices). The iᵗʰ
            vertex gets values <q(3i), q(3i + 1), q(3i + 2>. Each vertex is
            assumed to be measured and expressed in the mesh's frame M.
  @pre q.size == 3 * mesh().num_vertices(). */
  void UpdateVertexPositions(const Eigen::Ref<const VectorX<double>>& q) {
    mesh_->UpdateVertexPositions(q);
  }

  /* Returns the approximate signed distance field with which `this` is
   constructed. */
  const VolumeMeshFieldLinear<double, double>& signed_distance() const {
    return *signed_distance_;
  }

  /* Returns the volume mesh at reference configuration (i.e. the instant when
   * DeformableGeometry is constructed). */
  const VolumeMesh<double>& reference_mesh() const { return *reference_mesh_; }

 private:
  std::unique_ptr<internal::DeformableVolumeMesh<double>> mesh_;
  std::unique_ptr<VolumeMesh<double>> reference_mesh_;
  std::unique_ptr<VolumeMeshFieldLinear<double, double>> signed_distance_;
};

/* Defines a rigid geometry -- a rigid hydroelastic mesh repurposed to compute
 deformable vs. rigid contact, along with local data to keep track of the pose
 of the mesh. We need to locally store the pose of the
 mesh because right now we aren't relying on FCL's broadphase. */
class RigidGeometry {
 public:
  RigidGeometry() = default;

  explicit RigidGeometry(
      std::unique_ptr<internal::hydroelastic::RigidMesh> rigid_mesh)
      : rigid_mesh_(std::move(rigid_mesh)) {}

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RigidGeometry)

  const internal::hydroelastic::RigidMesh& rigid_mesh() const {
    DRAKE_DEMAND(rigid_mesh_ != nullptr);
    return *rigid_mesh_;
  }

  /* Updates the pose of the geometry in the world frame to the given pose,
   X_WG. */
  void set_pose_in_world(const math::RigidTransform<double>& X_WG) {
    X_WG_ = X_WG;
  }

  /* Returns the pose of the geometry in the world frame, X_WG. */
  const math::RigidTransform<double>& pose_in_world() const { return X_WG_; }

 private:
  copyable_unique_ptr<internal::hydroelastic::RigidMesh> rigid_mesh_;
  math::RigidTransform<double> X_WG_;
};

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
    return deformable_geometries_.at(id);
  }

  /* Returns the representation of the rigid geometry with the given `id`. */
  const RigidGeometry& rigid_geometry(GeometryId id) const {
    return rigid_geometries_.at(id);
  }

  bool is_rigid(GeometryId id) const {
    return rigid_geometries_.count(id) != 0;
  }

  bool is_deformable(GeometryId id) const {
    return deformable_geometries_.count(id) != 0;
  }

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
  void UpdateRigidWorldPose(GeometryId id, math::RigidTransform<double>& X_WG) {
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

  // The representations of all deformable geometries.
  std::unordered_map<GeometryId, DeformableGeometry> deformable_geometries_;

  // The representations of all rigid geometries.
  std::unordered_map<GeometryId, RigidGeometry> rigid_geometries_;
};

/* Generic interface for handling rigid Shapes. By default, we support all
 shapes that are supported by rigid hydroelastic. Unsupported geometries will
 return a std::nullopt.  */
template <typename Shape>
std::optional<RigidGeometry> MakeRigidRepresentation(
    const Shape& shape, const ProximityProperties& props) {
  std::optional<internal::hydroelastic::RigidGeometry> hydro_rigid_geometry =
      internal::hydroelastic::MakeRigidRepresentation(shape, props);
  if (!hydro_rigid_geometry) {
    static const logging::Warn log_once(
        "Rigid {} shapes are not currently supported for deformable "
        "contact; registration is allowed, but an error will be thrown "
        "during contact.",
        ShapeName(shape));
    return {};
  }
  auto surface_mesh = std::make_unique<TriangleSurfaceMesh<double>>(
      (*hydro_rigid_geometry).mesh());
  auto rigid_mesh = std::make_unique<internal::hydroelastic::RigidMesh>(
      std::move(surface_mesh));
  return RigidGeometry(std::move(rigid_mesh));
}

/* Half space is not supported for deformable contact at the moment as we
 require a surface mesh. */
std::optional<RigidGeometry> MakeRigidRepresentation(
    const HalfSpace&, const ProximityProperties&) {
  static const logging::Warn log_once(
      "Half spaces are not currently supported for deformable contact; "
      "registration is allowed, but an error will be thrown "
      "during contact.");
  return {};
}

/* Generic interface for handling unsupported deformable Shapes. Unsupported
 geometries will return a std::nullopt.  */
template <typename Shape>
std::optional<DeformableGeometry> MakeDeformableRepresentation(
    const Shape& shape, const ProximityProperties&) {
  static const logging::Warn log_once(
      "Deformable {} shapes are not currently supported for deformable "
      "contact; registration is allowed, but an error will be thrown during "
      "contact.",
      ShapeName(shape));
  return {};
}

// TODO(xuchenhan-tri): Find a way to ensure that the mesh used for proximity
//  queries are the same as that used in the FEM model.
/* Creates a deformable sphere (assuming the proximity properties have
 sufficient information). Requires the ('hydroelastic', 'resolution_hint')
 property. */
std::optional<DeformableGeometry> MakeDeformableRepresentation(
    const Sphere& sphere, const ProximityProperties& props);

/* Creates a deformable box (assuming the proximity properties have sufficient
 information). Requires the ('hydroelastic', 'resolution_hint') property. */
std::optional<DeformableGeometry> MakeDeformableRepresentation(
    const Box& box, const ProximityProperties& props);

}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
