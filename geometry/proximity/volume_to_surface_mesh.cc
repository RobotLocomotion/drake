#include "drake/geometry/proximity/volume_to_surface_mesh.h"

#include <map>
#include <set>
#include <unordered_map>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/geometry/proximity/sorted_triplet.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

std::vector<std::array<int, 3>> IdentifyBoundaryFaces(
    const std::vector<VolumeElement>& tetrahedra,
    std::vector<std::pair<int, int>>* element_indices) {
  // We want to identify a triangle ABC from all six permutations of A,B,C
  // (i.e., ABC, ACB, BAC, BCA, CAB, CBA), so we use SortedTriplet(A,B,C)
  // as a unique representation of all permutations.
  //     A triangular face of a tetrahedron is either shared with another
  // tetrahedron or on the boundary surface of `volume`.
  //     We maintain a map from SortedTriplet(A,B,C) to array{A,B,C} as we
  // go through each triangular face of each tetrahedron. The SortedTriplet
  // gives us the unique representation, and the array{A,B,C} gives us the
  // appropriate winding or orientation.
  //     For each face of each tetrahedron, if its SortedTriplet is already in
  // the map, we remove the entry from the map because the face is shared with
  // a previous tetrahedron. Otherwise, we insert the pair
  // {SortedTriplet(A,B,C), array{A,B,C}} into the map.
  //     In the end, the entries in the map correspond to the triangular
  // faces on the boundary surface of the volume.
  //     We use `map` instead of `unordered_map` so that we get the same
  // result on different computers, operating systems, or compilers. It will
  // help with repeatability between different users on different platforms.
  // The canonical order of the entries in the map is also useful in
  // debugging. However, `map` is slower, and we may change to
  // `unordered_map` later if `map` is too slow.
  std::map<SortedTriplet<int>,
           std::pair<std::array<int, 3>, std::pair<int, int>>>
      face_map;

  auto insert_or_erase = [&face_map](int v0, int v1, int v2, int element_index,
                                     int local_index) {
    SortedTriplet<int> sorted(v0, v1, v2);
    auto find = face_map.find(sorted);
    if (find != face_map.end()) {
      face_map.erase(find);
    } else {
      face_map.emplace(
          sorted, std::make_pair(std::array<int, 3>{v0, v1, v2},
                                 std::make_pair(element_index, local_index)));
    }
  };

  // According to VolumeElement, the first three vertices of a tetrahedron
  // define a triangle with its right-handed normal pointing inwards. The
  // fourth vertex is on the positive side of this first triangle. An example
  // of the four vertices v0,v1,v2,v3 of such a tetrahedron is shown in this
  // picture:
  //
  //      +Z
  //       |
  //       v3
  //       |
  //       |
  //     v0+------v2---+Y
  //      /
  //     /
  //   v1
  //   /
  // +X
  //
  // From the picture above, we can see that each of the following four
  // triangular faces:
  // v1 v2 v3
  // v3 v2 v0
  // v2 v1 v0
  // v1 v3 v0
  // has its right-handed normal pointing outwards from the tetrahedron.
  const int tetrahedron_faces[4][4] = {
      // clang-format off
      {1, 2, 3, 0},
      {3, 2, 0, 1},
      {1, 3, 0, 2},
      {2, 1, 0, 3}
      // clang-format on
  };

  for (int element_index = 0; element_index < ssize(tetrahedra);
       ++element_index) {
    const VolumeElement& tetrahedron = tetrahedra[element_index];
    for (const auto& face_vertices : tetrahedron_faces) {
      insert_or_erase(tetrahedron.vertex(face_vertices[0]),
                      tetrahedron.vertex(face_vertices[1]),
                      tetrahedron.vertex(face_vertices[2]), element_index,
                      face_vertices[3]);
    }
  }

  std::vector<std::array<int, 3>> boundary;
  boundary.reserve(face_map.size());
  if (element_indices) {
    element_indices->clear();
    element_indices->reserve(face_map.size());
  }
  for (const auto& [key, pair] : face_map) {
    boundary.emplace_back(pair.first);
    if (element_indices) {
      element_indices->emplace_back(pair.second);
    }
  }

  return boundary;
}

std::vector<int> CollectUniqueVertices(
    const std::vector<std::array<int, 3>>& faces) {
  // We use `set` instead of `unordered_set` so that we get the same
  // result on different computers, operating systems, or compilers. It will
  // help with repeatability between different users on different platforms.
  // The canonical order of the vertices is also useful in debugging.
  // However, `set` is slower, and we may change to `unordered_set` later if
  // `set` is too slow.
  std::set<int> vertex_set;
  for (const auto& face : faces) {
    for (const auto& vertex : face) {
      vertex_set.insert(vertex);
    }
  }
  return std::vector<int>(vertex_set.begin(), vertex_set.end());
}

template <class T>
TriangleSurfaceMesh<T>
ConvertVolumeToSurfaceMeshWithBoundaryVerticesAndElementMap(
    const VolumeMesh<T>& volume, std::vector<int>* boundary_vertices_out,
    std::vector<std::pair<int, int>>* element_indices_out) {
  const std::vector<std::array<int, 3>> boundary_faces =
      internal::IdentifyBoundaryFaces(volume.tetrahedra(), element_indices_out);

  std::vector<int> boundary_vertices =
      internal::CollectUniqueVertices(boundary_faces);

  std::vector<Vector3<T>> surface_vertices;
  surface_vertices.reserve(boundary_vertices.size());

  // Map from an index into the volume mesh's vertices to the resulting
  // surface mesh's vertices.
  std::unordered_map<int, int> volume_to_surface;
  for (int i = 0; i < static_cast<int>(boundary_vertices.size()); ++i) {
    surface_vertices.emplace_back(volume.vertex(boundary_vertices[i]));
    volume_to_surface.emplace(boundary_vertices[i], i);
  }

  std::vector<SurfaceTriangle> surface_faces;
  surface_faces.reserve(boundary_faces.size());
  for (const auto& face_vertices : boundary_faces) {
    surface_faces.emplace_back(volume_to_surface.at(face_vertices[0]),
                               volume_to_surface.at(face_vertices[1]),
                               volume_to_surface.at(face_vertices[2]));
  }

  if (boundary_vertices_out != nullptr) {
    *boundary_vertices_out = std::move(boundary_vertices);
  }
  return TriangleSurfaceMesh<T>(std::move(surface_faces),
                                std::move(surface_vertices));
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&ConvertVolumeToSurfaceMeshWithBoundaryVerticesAndElementMap<T>));

}  // namespace internal
}  // namespace geometry
}  // namespace drake
