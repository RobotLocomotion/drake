#pragma once

#include <array>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/*
 Identify the triangular boundary faces of a tetrahedral volume mesh.
 @param tetrahedra      The tetrahedra of the mesh.
 @param element_indices An optional pointer to vector of pair of indices. The
 first index of the pair being the index of the volume element that each
 boundary face came from. The second index of the pair being the local index of
 the surface face in the volume element.
 @return             The boundary faces, each of which is represented as an
                     array of three indices of vertices of the volume mesh.
                     Each face has its right-handed normal pointing outwards
                     from the volume mesh.
 @pre  1. The volume mesh cannot be a tetrahedron soup. If two tetrahedra
          share a face, they must share vertex indices and not index into
          duplicate vertices.
       2. Any given face is shared by one or two tetrahedra only.
 */
std::vector<std::array<int, 3>> IdentifyBoundaryFaces(
    const std::vector<VolumeElement>& tetrahedra,
    std::vector<std::pair<int, int>>* element_indices = nullptr);

/*
 Collects unique vertices from faces of a volume mesh. Each vertex shared by
 multiple faces is reported once. Each face is represented as an array of
 three vertices of a volume mesh.
 @param[in] faces
 @return    vertices used by the faces. The vertex indices are sorted in
            increasing order.
 */
std::vector<int> CollectUniqueVertices(
    const std::vector<std::array<int, 3>>& faces);

/*
 Implements the public API ConvertVolumeToSurfaceMesh() with optional return
 of the boundary vertices that we can use internally and (optionally) the
 mapping from TriangleSurfaceMesh elements to the VolumeMesh element containing
 it in the original mesh. The returned integer indices are sorted in increasing
 order and refer to vertices in the input `volume` mesh.
 */
template <class T>
TriangleSurfaceMesh<T>
ConvertVolumeToSurfaceMeshWithBoundaryVerticesAndElementMap(
    const VolumeMesh<T>& volume,
    std::vector<int>* boundary_vertices_out = nullptr,
    std::vector<std::pair<int, int>>* element_map = nullptr);

template <class T>
TriangleSurfaceMesh<T> ConvertVolumeToSurfaceMeshWithBoundaryVertices(
    const VolumeMesh<T>& volume,
    std::vector<int>* boundary_vertices_out = nullptr) {
  return ConvertVolumeToSurfaceMeshWithBoundaryVerticesAndElementMap(
      volume, boundary_vertices_out);
}

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
  return internal::ConvertVolumeToSurfaceMeshWithBoundaryVerticesAndElementMap(
      volume, nullptr, nullptr);
}

}  // namespace geometry
}  // namespace drake
