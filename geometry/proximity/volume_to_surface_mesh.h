#pragma once

#include <array>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/**
 Identify the triangular boundary faces of a tetrahedral volume mesh.
 @param tetrahedra   The tetrahedra of the mesh.
 @return             The boundary faces, each of which is represented as an
                     array of three indices of vertices of the volume mesh.
                     Each face has its right-handed normal pointing outwards
                     from the volume mesh.
 @pre  1. The volume mesh cannot be a tetrahedron soup. If two tetrahedra
          share a face, they must share vertex indices and not index into
          duplicate vertices.
       2. Any given face is shared by one or two tetrahedra only.
 */
std::vector<std::array<VolumeVertexIndex, 3>> IdentifyBoundaryFaces(
    const std::vector<VolumeElement>& tetrahedra);

/**
 Collects unique vertices from faces of a volume mesh. Each vertex shared by
 multiple faces is reported once. Each face is represented as an array of
 three vertices of a volume mesh.
 @param[in] faces
 @return    vertices used by the faces.
 */
std::vector<VolumeVertexIndex> CollectUniqueVertices(
    const std::vector<std::array<VolumeVertexIndex, 3>>& faces);

/**
 Converts a tetrahedral volume mesh to a triangulated surface mesh of the
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
 @tparam T      The underlying scalar type for coordinates, e.g., double
                or AutoDiffXd. Must be a valid Eigen scalar.
 */
template <class T>
SurfaceMesh<T> ConvertVolumeToSurfaceMesh(const VolumeMesh<T>& volume) {
  const std::vector<std::array<VolumeVertexIndex, 3>> boundary_faces =
      IdentifyBoundaryFaces(volume.tetrahedra());

  const std::vector<VolumeVertexIndex> boundary_vertices =
      CollectUniqueVertices(boundary_faces);

  std::vector<SurfaceVertex<T>> surface_vertices;
  surface_vertices.reserve(boundary_vertices.size());
  std::unordered_map<VolumeVertexIndex, SurfaceVertexIndex> volume_to_surface;
  for (SurfaceVertexIndex i(0); i < boundary_vertices.size(); ++i) {
    surface_vertices.emplace_back(volume.vertex(boundary_vertices[i]).r_MV());
    volume_to_surface.emplace(boundary_vertices[i], i);
  }

  std::vector<SurfaceFace> surface_faces;
  surface_faces.reserve(boundary_faces.size());
  for (const auto& face_vertices : boundary_faces) {
    surface_faces.emplace_back(volume_to_surface.at(face_vertices[0]),
                               volume_to_surface.at(face_vertices[1]),
                               volume_to_surface.at(face_vertices[2]));
  }

  return SurfaceMesh<T>(std::move(surface_faces), std::move(surface_vertices));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
