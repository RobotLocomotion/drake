#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/deformable_mesh_with_bvh.h"
#include "drake/geometry/proximity/hydroelastic_internal.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {

// TODO(xuchenhan-tri): Consider supporting AutoDiffXd.
/* Definition of a deformable geometry for contact evaluations. To be
 considered as deformable, a geometry must be associated with both:
   - a deformable volume mesh, and
   - an approximate signed distance field in the interior of the mesh. */
class DeformableGeometry {
 public:
  DeformableGeometry(const DeformableGeometry& other);
  DeformableGeometry& operator=(const DeformableGeometry& other);
  DeformableGeometry(DeformableGeometry&&) = default;
  DeformableGeometry& operator=(DeformableGeometry&&) = default;

  /* Constructs a deformable geometry from the given mesh. Also computes an
   approximate signed distance field for the mesh. */
  explicit DeformableGeometry(VolumeMesh<double> mesh);

  // TODO(xuchenhan-tri): Consider adding another constructor that takes in both
  // a mesh and a precomputed (approximated) sign distance field.

  /* Returns the volume mesh representation of the deformable geometry at
   current configuration. */
  const DeformableVolumeMeshWithBvh<double>& deformable_mesh() const {
    return *deformable_mesh_;
  }

  /* Updates the vertex positions of the underlying deformable mesh.
  @param q  A vector of 3N values (where this mesh has N vertices). The iᵗʰ
            vertex gets values <q(3i), q(3i + 1), q(3i + 2)>. Each vertex is
            assumed to be measured and expressed in the mesh's frame M.
  @pre q.size() == 3 * deformable_mesh().num_vertices(). */
  void UpdateVertexPositions(const Eigen::Ref<const VectorX<double>>& q) {
    DRAKE_DEMAND(q.size() == 3 * deformable_mesh().mesh().num_vertices());
    deformable_mesh_->UpdateVertexPositions(q);
  }

  /* Returns the approximate signed distance field (sdf) for the deformable
   geometry evaluated with the deformable mesh at its *current* configuration.
   More specifically, the sdf value at each vertex is equal to the exact value
   (to the accuracy of the distance query algorithm) in their reference
   configuration. The values in the interior of the mesh are linearly
   interpolated from vertex values.
   @warn The result may no longer be valid after calls to
   UpdateVertexPositions(). Hence, the result should be used and discarded
   instead of kept around. */
  const VolumeMeshFieldLinear<double, double>& CalcSignedDistanceField() const;

 private:
  std::unique_ptr<DeformableVolumeMeshWithBvh<double>> deformable_mesh_;
  /* Note: we don't provide an accessor to `signed_distance_field_` as it may be
   invalidated by calls to `UpdateVertexPositions()`. Instead, we provide
   `CalcSignedDistanceField()` that guarantees to return the up-to-date mesh
   field. */
  std::unique_ptr<VolumeMeshFieldLinear<double, double>> signed_distance_field_;
};

/* Defines a rigid geometry -- a rigid hydroelastic mesh repurposed to compute
 deformable vs. rigid contact, along with local data to keep track of the pose
 of the mesh. We need to locally store the pose of the mesh because right now we
 aren't relying on FCL's broadphase. */
class RigidGeometry {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RigidGeometry);

  explicit RigidGeometry(
      std::unique_ptr<internal::hydroelastic::RigidMesh> rigid_mesh)
      : rigid_mesh_(std::move(rigid_mesh)) {}

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
 space) can choose to opt out. Geometries not supported by rigid hydroelastics
 will return a std::nullopt. The rigid mesh created upon a successful creation
 of RigidGeometry will be the same mesh as used for rigid hydroelastics. */
template <typename Shape>
std::optional<RigidGeometry> MakeRigidRepresentation(
    const Shape& shape, const ProximityProperties& props) {
  std::optional<internal::hydroelastic::RigidGeometry> hydro_rigid_geometry =
      internal::hydroelastic::MakeRigidRepresentation(shape, props);
  if (!hydro_rigid_geometry || hydro_rigid_geometry->is_half_space()) {
    return {};
  }
  /* RigidGeometry is documented as having a mesh or having a half space. We've
   excluded the latter, so we know we have a mesh. */
  return RigidGeometry(hydro_rigid_geometry->release_mesh());
}

}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
