#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_assert.h"
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
 considered as deformable, a geometry must be associated with:
   - a deformable volume mesh,
   - an approximate signed distance field in the interior of the volume mesh,
     and
   - the surface triangle mesh of the volume mesh. */
class DeformableGeometry {
 public:
  DeformableGeometry(const DeformableGeometry& other);
  DeformableGeometry& operator=(const DeformableGeometry& other);
  DeformableGeometry(DeformableGeometry&&) = default;
  DeformableGeometry& operator=(DeformableGeometry&&) = default;

  /* Constructs a deformable geometry from the given meshes. Also computes an
   approximate signed distance field for the mesh.
   @param[in] volume_mesh   The volume mesh representation of the geometry.
   @param[in] surface_mesh  The surface of `volume_mesh`.
   @param[in] surface_index_to_volume_index
                            Mapping from surface index to volume index. The iᵗʰ
                            entry is the index of the volume mesh vertex that
                            corresponds to the iᵗʰ surface vertex. */
  DeformableGeometry(VolumeMesh<double> volume_mesh,
                     TriangleSurfaceMesh<double> surface_mesh,
                     std::vector<int> surface_index_to_volume_index,
                     std::vector<int> surface_tri_to_volume_tet);

  // TODO(xuchenhan-tri): Consider adding another constructor that takes in both
  // a mesh and a precomputed (approximated) sign distance field.

  /* Returns the volume mesh representation of the deformable geometry at
   current configuration. */
  const DeformableVolumeMeshWithBvh<double>& deformable_volume() const {
    return *deformable_volume_;
  }

  /* Returns the surface mesh representation of the deformable geometry at
   current configuration. */
  const DeformableSurfaceMeshWithBvh<double>& deformable_surface() const {
    return *deformable_surface_;
  }

  /* Returns the mapping from surface index to volume index. The iᵗʰ entry is
   the index of the volume mesh vertex that corresponds to the iᵗʰ surface
   vertex. */
  const std::vector<int>& surface_index_to_volume_index() const {
    return surface_index_to_volume_index_;
  }

  /* Returns the mapping from surface triangle to volume tetrahehdron. The iᵗʰ
   entry is the index of the volume mesh tetrahedron that corresponds to the iᵗʰ
   surface triangle. */
  const std::vector<int>& surface_tri_to_volume_tet() const {
    return surface_tri_to_volume_tet_;
  }

  /* Updates the vertex positions of the deformable geometry.
  @param q_volume  A vector of 3N values (where the volume mesh has N vertices).
                   The iᵗʰ vertex in the volume mesh gets values
                   <q(3i), q(3i + 1), q(3i + 2)>. Each vertex is assumed to be
                   measured and expressed in the mesh's frame M.
  @param q_surface Similar to `q_volume`, but provides the vertex positions of
                   the surface mesh.
  @pre q_volume.size() == 3 * deformable_volume().mesh().num_vertices().
  @pre q_surface.size() == 3 * deformable_surface().mesh(). num_vertices(). */
  void UpdateVertexPositions(
      const Eigen::Ref<const VectorX<double>>& q_volume,
      const Eigen::Ref<const VectorX<double>>& q_surface) {
    DRAKE_DEMAND(q_volume.size() ==
                 3 * deformable_volume().mesh().num_vertices());
    DRAKE_DEMAND(q_surface.size() ==
                 3 * deformable_surface().mesh().num_vertices());
    deformable_volume_->UpdateVertexPositions(q_volume);
    deformable_surface_->UpdateVertexPositions(q_surface);
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

  /* Returns the axis-aligned bounding box of the geometry in the world frame.
   */
  const Aabb& aabb_in_world() const {
    return deformable_volume_->bvh().root_node().bv();
  }

 private:
  std::unique_ptr<DeformableVolumeMeshWithBvh<double>> deformable_volume_;
  std::unique_ptr<DeformableSurfaceMeshWithBvh<double>> deformable_surface_;
  std::vector<int> surface_index_to_volume_index_;
  std::vector<int> surface_tri_to_volume_tet_;
  /* Note: we don't provide an accessor to `signed_distance_field_` as it may be
   invalidated by calls to `UpdateVertexPositions()`. Instead, we provide
   `CalcSignedDistanceField()` that guarantees to return the up-to-date mesh
   field. */
  std::unique_ptr<VolumeMeshFieldLinear<double, double>> signed_distance_field_;
};

// TODO(xuchenhan-tri): Rename this class to NonDeformableGeometry. The name
//  "rigid" is too overloaded.
/* Defines a non-deformable geometry -- a compliant hydroelastic mesh repurposed
 to compute deformable vs. non-deformable contact, along with local data to keep
 track of the pose of the mesh. We need to locally store the pose of the mesh
 because right now we aren't relying on FCL's broadphase. */
class RigidGeometry {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RigidGeometry);

  explicit RigidGeometry(std::unique_ptr<internal::hydroelastic::SoftMesh> mesh)
      : mesh_(std::move(mesh)) {}

  const internal::hydroelastic::SoftMesh& mesh() const {
    DRAKE_DEMAND(mesh_ != nullptr);
    return *mesh_;
  }

  /* Updates the pose of the geometry in the world frame to the given pose. */
  void set_pose_in_world(const math::RigidTransform<double>& X_WG) {
    X_WG_ = X_WG;
  }

  /* Returns the pose of the geometry in the world frame. */
  const math::RigidTransform<double>& pose_in_world() const { return X_WG_; }

 private:
  copyable_unique_ptr<internal::hydroelastic::SoftMesh> mesh_;
  math::RigidTransform<double> X_WG_;
};

/* Generic interface for handling rigid Shapes. By default, we support all
 shapes that are supported by compliant hydroelastic. Unsupported shapes (e.g.
 half space) can choose to opt out. Geometries not supported by compliant
 hydroelastics will return a std::nullopt. The mesh created upon a successful
 creation of RigidGeometry will be the same mesh as used for compliant
 hydroelastics. */
template <typename Shape>
std::optional<RigidGeometry> MakeMeshRepresentation(
    const Shape& shape, const ProximityProperties& props) {
  std::optional<internal::hydroelastic::SoftGeometry> compliant_hydro_geometry =
      internal::hydroelastic::MakeSoftRepresentation(shape, props);
  // TODO(xuchenhan-tri): Support half space.
  if (!compliant_hydro_geometry || compliant_hydro_geometry->is_half_space()) {
    return {};
  }
  /* hydroelastic::SoftGeometry is documented as having a mesh or having a half
   space. We've excluded the latter, so we know we have a mesh. */
  // TODO(xuchenhan-tri): consider allowing SoftMesh to release its mesh to
  // prevent copying here.
  auto mesh =
      std::make_unique<VolumeMesh<double>>(compliant_hydro_geometry->mesh());
  std::unique_ptr<VolumeMeshFieldLinear<double, double>> field =
      compliant_hydro_geometry->pressure_field().CloneAndSetMesh(mesh.get());
  return RigidGeometry(std::make_unique<internal::hydroelastic::SoftMesh>(
      std::move(mesh), std::move(field)));
}

}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
