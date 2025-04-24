#pragma once

#include <array>
#include <vector>

#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* Unique identifier for a face of a tetrahedron in a volume mesh. The mesh
 isn't stored. The identifier may apply to arbitrary meshes, describing
 radically different triangles. The caller is responsible for maintaining the
 association between a %TetFace and the volume mesh to which it refer. */
struct TetFace {
  /* The index of the tetrahedron to which the face belongs. */
  int tet_index{};
  /* The _local_ index of the face in the indexed tet. Given a tet, the
   actual triangle can be inferred as a triplet of tet-local vertex indices:

   | face_index | triple of tet vertex indices |
   | :========: | :==========================: |
   |     0      |       1, 2, 3                |
   |     1      |       3, 2, 0                |
   |     2      |       1, 3, 0                |
   |     3      |       2, 1, 0                | */
  int face_index{};

  bool operator==(const TetFace&) const;  // = default;
};

/* Identifies the triangular boundary faces of a tetrahedral volume mesh.
 @param tetrahedra       The tetrahedra of the mesh.
 @param element_indices  An optional pointer to vector of TetFace identifiers.
                         The vector will be cleared and the iᵗʰ entry in
                         `element_indices` corresponds to the iᵗʰ entry in the
                         returned vector.
 @return The boundary faces, each of which is represented as an array of three
         indices of vertices of the volume mesh. Each face has its right-handed
         normal pointing outwards from the volume mesh.
 @pre  1. The volume mesh cannot be a tetrahedron soup. If two tetrahedra
          share a face, they must share vertex indices and not index into
          duplicate vertices.
       2. Any given face is shared by one or two tetrahedra only. */
std::vector<std::array<int, 3>> IdentifyBoundaryFaces(
    const std::vector<VolumeElement>& tetrahedra,
    std::vector<TetFace>* element_indices = nullptr);

/* Collects unique vertices from `faces` of a volume mesh. Each vertex shared by
 multiple faces is reported once. Each face is represented as an array of
 three vertices of a volume mesh.
 @param[in] faces  The faces, from which the vertex set is constructed.
 @return    The set of vertices referenced by `faces`. The vertex indices are
            sorted in increasing order. */
std::vector<int> CollectUniqueVertices(
    const std::vector<std::array<int, 3>>& faces);

/* Implements the public API ConvertVolumeToSurfaceMesh() with optional return
 of:
   - the boundary vertices; a map between vertices in the surface mesh to
     vertices in the volume mesh. The iᵗʰ entry in `boundary_vertices_out`
     contains the index of the volume vertex corresponding to the iᵗʰ vertex in
     the surface mesh.
   - a record of which tet-face each surface triangle came from. The iᵗʰ entry
     in `tri_to_tet_face_map` corresponds to the iᵗʰ triangle in the returned
     surface mesh. */
template <class T>
TriangleSurfaceMesh<T> ConvertVolumeToSurfaceMeshWithBoundaryVertices(
    const VolumeMesh<T>& volume,
    std::vector<int>* boundary_vertices_out = nullptr,
    std::vector<TetFace>* tri_to_tet_face_map = nullptr);

/* Overload that returns the boundary vertices as well as the tet indices of the
 surface triangles. Note that `tri_to_tet_map` contains the same information as
 `tri_to_tet_face_map`, just with the face information dropped. */
template <class T>
TriangleSurfaceMesh<T> ConvertVolumeToSurfaceMeshWithBoundaryVertices(
    const VolumeMesh<T>& volume, std::vector<int>* boundary_vertices_out,
    std::vector<int>* tri_to_tet_map);

}  // namespace internal

/** Converts a tetrahedral volume mesh to a triangulated surface mesh of the
 boundary surface of the volume.
 @param volume  The tetrahedral volume mesh, whose vertex positions are
                measured and expressed in some frame E.
 @return        The triangulated surface mesh, whose vertex positions are
                measured and expressed in the same frame E of the volume mesh.
 @pre           The vertices of the volume mesh are unique. Adjacent
                tetrahedra share the same vertices, instead of repeating the
                vertices with the same coordinates. Otherwise, the returned
                surface mesh will have extra triangles in addition to the
                boundary triangles of the volume.
 @tparam_nonsymbolic_scalar */
template <class T>
TriangleSurfaceMesh<T> ConvertVolumeToSurfaceMesh(const VolumeMesh<T>& volume) {
  return internal::ConvertVolumeToSurfaceMeshWithBoundaryVertices(volume);
}

}  // namespace geometry
}  // namespace drake
