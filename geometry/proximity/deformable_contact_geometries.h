#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
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

/* Definition of a deformable body's geometry at the reference configuration.
 It includes a volume mesh and a scalar field approximating the signed distance
 to the surface of the mesh defined on the interior of the geometry. This class
 is similar to geometry::internal::hydroelastic::SoftMesh with two distinctions:
 1. This class doesn't provide a bounding volume hierarchy.
 2. This class calculates an approximate signed distance field (meters, negative
    inside) instead of taking a prescribed pressure field (Pascals, positive
    inside). */
class ReferenceDeformableGeometry : public ShapeReifier {
 public:
  // TODO(xuchenhan-tri): Consider if it's possible to take a const pointer to
  // the mesh instead of owning a copy of the mesh.
  /* Constructs a deformable geometry at reference configuration with the given
  `shape` and its spatial discretization `mesh`. An internal approximate signed
  distance field in the `mesh` (to the surface of `shape`) is created at
  construction.
  @param shape  The shape of the deformable geometry in reference configuration.
  @param mesh   A reasonable tetrahedral tessellation of the given `shape`. */
  ReferenceDeformableGeometry(const Shape& shape, VolumeMesh<double> mesh);

  /* Custom copy assign and construct. */
  ReferenceDeformableGeometry& operator=(const ReferenceDeformableGeometry&);
  ReferenceDeformableGeometry(const ReferenceDeformableGeometry&);
  /* Default move assign and construct. */
  ReferenceDeformableGeometry(ReferenceDeformableGeometry&&) = default;
  ReferenceDeformableGeometry& operator=(ReferenceDeformableGeometry&&) =
      default;

  /* Returns the volume mesh representation of the deformable geometry at
   reference configuration. */
  const VolumeMesh<double>& mesh() const {
    DRAKE_DEMAND(mesh_ != nullptr);
    return *mesh_;
  }

  /* Returns the approximate signed distance field (sdf) to the surface of the
   deformable geometry in the reference configuration. More specifically, the
   sdf value at each vertex is exact (to the accuracy of the distance query
   algorithm), and the values in the interior of the mesh are linearly
   interpolated from vertex values. */
  const VolumeMeshFieldLinear<double, double>& signed_distance_field() const {
    DRAKE_DEMAND(signed_distance_field_ != nullptr);
    return *signed_distance_field_;
  }

 private:
  /* Data to be used during reification. It is passed as the `user_data`
   parameter in the ImplementGeometry API. */
  struct ReifyData {
    std::vector<double> signed_distance;
  };

  std::vector<double> ComputeSignedDistanceOfVertices(const Shape& shape);

  using ShapeReifier::ImplementGeometry;
  void ImplementGeometry(const Sphere& sphere, void* user_data) override;
  void ImplementGeometry(const Box& box, void* user_data) override;

  std::unique_ptr<VolumeMesh<double>> mesh_{nullptr};
  std::unique_ptr<VolumeMeshFieldLinear<double, double>> signed_distance_field_{
      nullptr};
};

/* Definition of a deformable geometry for contact implementations. To be a
 deformable geometry, a shape must be associated with both:
   - a deformable volume mesh, and
   - an approximate signed distance field in the interior of the mesh. */
class DeformableGeometry {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableGeometry)

  /* Constructs a deformable geometry. */
  DeformableGeometry(const Shape& shape, VolumeMesh<double> mesh)
      : reference_geometry_(shape, mesh), deformable_mesh_(std::move(mesh)) {}

  /* Returns the volume mesh representation of the deformable geometry at
   current configuration. */
  const DeformableVolumeMesh<double>& deformable_mesh() const {
    return deformable_mesh_;
  }

  /* Updates the vertex positions of the underlying deformable mesh.
  @param q  A vector of 3N values (where this mesh has N vertices). The iᵗʰ
            vertex gets values <q(3i), q(3i + 1), q(3i + 2)>. Each vertex is
            assumed to be measured and expressed in the mesh's frame M.
  @pre q.size() == 3 * deformable_mesh().num_vertices(). */
  void UpdateVertexPositions(const Eigen::Ref<const VectorX<double>>& q) {
    DRAKE_DEMAND(q.size() == 3 * deformable_mesh().mesh().num_vertices());
    deformable_mesh_.UpdateVertexPositions(q);
  }

  /* Returns the approximate signed distance field (sdf) for the deformable
   geometry. More specifically, the sdf value at each vertex is equal to the
   exact value (to the accuracy of the distance query algorithm) in their
   reference configuration. The values in the interior of the mesh are linearly
   interpolated from vertex values. */
  const VolumeMeshFieldLinear<double, double>& signed_distance_field() const {
    return reference_geometry_.signed_distance_field();
  }

 private:
  ReferenceDeformableGeometry reference_geometry_;
  DeformableVolumeMesh<double> deformable_mesh_;
};

/* Defines a rigid geometry -- a rigid hydroelastic mesh repurposed to compute
 deformable vs. rigid contact, along with local data to keep track of the pose
 of the mesh. We need to locally store the pose of the mesh because right now we
 aren't relying on FCL's broadphase. */
class RigidGeometry {
 public:
  explicit RigidGeometry(
      std::unique_ptr<internal::hydroelastic::RigidMesh> rigid_mesh)
      : rigid_mesh_(std::move(rigid_mesh)) {}

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RigidGeometry)

  const internal::hydroelastic::RigidMesh& rigid_mesh() const {
    DRAKE_DEMAND(rigid_mesh_ != nullptr);
    return *rigid_mesh_;
  }

  /* Updates the pose of the geometry in the world frame to the given pose. */
  void set_pose_in_world(const math::RigidTransform<double>& X_WG) {
    X_WG_ = X_WG;
  }

  /* Returns the pose of the geometry in the world frame. */
  const math::RigidTransform<double>& pose_in_world() const { return X_WG_; }

 private:
  copyable_unique_ptr<internal::hydroelastic::RigidMesh> rigid_mesh_;
  math::RigidTransform<double> X_WG_;
};

/* Generic interface for handling rigid Shapes. By default, we support all
 shapes that are supported by rigid hydroelastic. Unsupported shapes (e.g. half
 space) can choose to opt out. Unsupported geometries will return a
 std::nullopt. The rigid mesh created upon a successful creation of
 RigidGeometry will be the same mesh as used for rigid hydroelastics. */
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
    const HalfSpace&, const ProximityProperties&);

}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
