#pragma once

/** @file
 Extend the helpers for computing hydroelastic contact surface (see
 contact_surface_utility.h) to deformables. We still produce ContactSurface
 in the context of deformables with additional data.
 */

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* A MeshBuilder type to build a contact surface on a deformable geometry.
 It shares the same APIs with other MeshBuilder types, e.g., TriMeshBuilder
 and PolyMeshBuilder.

 It is similar to PolyMeshBuilder (see contact_surface_utility.h), which
 is used to build ContactSurface for hydroelastics. Furthermore, it also
 builds a list of tetrahedron index for each contact polygon for deformable
 simulation. */
template <typename T>
class DeformableContactBuilder {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableContactBuilder);

  using ScalarType = T;
  using MeshType = PolygonSurfaceMesh<T>;
  using FieldType = PolygonSurfaceMeshFieldLinear<T, T>;

  DeformableContactBuilder() {}

  /* Same as PolyMeshBuilder::AddVertex() with the interpretation of
   field_value as the approximated signed distance. */
  int AddVertex(const Vector3<T>& p_BV, const T& field_value);

  /* Same as PolyMeshBuilder::AddPolygon() and also collects the
   tetrahedron index of the deformable mesh that was used in computing the
   polygon.
   @note It takes only one tetrahedron to support rigid-deformable contact.
         It does not support other contact mode yet.
   @pre tetrahedron_index is at least 0 and less than the number of
        tetrahedra in the deformable mesh. */
  int AddPolygon(const std::vector<int>& polygon_vertices,
                 const Vector3<T>& nhat_B, const Vector3<T>& grad_e_MN_B,
                 int tetrahedron_index);

  /* Same as PolyMeshBuilder */
  int num_vertices() const;
  /* Same as PolyMeshBuilder */
  int num_faces() const;

  /* Same as PolyMeshBuilder */
  std::pair<std::unique_ptr<MeshType>, std::unique_ptr<FieldType>>
  MakeMeshAndField();

  /* Returns the list of tetrahedron indices accumulated so far. Call this
   function in order to build deformable contact data after the
   rigid-deformable mesh intersection has completed.
   */
  const std::vector<int>& tetrahedron_index_of_polygons() {
    return tetrahedron_index_of_polygons_;
  }

 private:
  PolyMeshBuilder<T> poly_mesh_builder_;
  std::vector<int> tetrahedron_index_of_polygons_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
