#pragma once

#include <array>
#include <vector>

#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/*
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
std::vector<std::array<int, 3>> IdentifyBoundaryFaces(
    const std::vector<VolumeElement>& tetrahedra);

/*
 Collects unique vertices from faces of a volume mesh. Each vertex shared by
 multiple faces is reported once. Each face is represented as an array of
 three vertices of a volume mesh.
 @param[in] faces
 @return    vertices used by the faces.
 */
std::vector<int> CollectUniqueVertices(
    const std::vector<std::array<int, 3>>& faces);

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
TriangleSurfaceMesh<T> ConvertVolumeToSurfaceMesh(const VolumeMesh<T>& volume);

}  // namespace geometry
}  // namespace drake
