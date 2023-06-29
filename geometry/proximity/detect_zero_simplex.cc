#include "drake/geometry/proximity/detect_zero_simplex.h"

#include <algorithm>
#include <array>
#include <set>
#include <unordered_set>

#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

namespace {

std::unordered_set<int> GetBoundaryVertexSet(const VolumeMesh<double>& mesh_M) {
  std::vector<int> boundary_vertex_vector =
      CollectUniqueVertices(IdentifyBoundaryFaces(mesh_M.tetrahedra()));
  return {boundary_vertex_vector.begin(), boundary_vertex_vector.end()};
}

std::set<SortedTriplet<int>> GetBoundaryFaceSet(
    const VolumeMesh<double>& mesh_M) {
  const std::vector<std::array<int, 3>> boundary_unordered_faces =
      IdentifyBoundaryFaces(mesh_M.tetrahedra());
  std::vector<SortedTriplet<int>> boundary_ordered_faces(
      boundary_unordered_faces.size());
  std::transform(boundary_unordered_faces.begin(),
                 boundary_unordered_faces.end(), boundary_ordered_faces.begin(),
                 [](const std::array<int, 3>& f) {
                   return SortedTriplet<int>(f[0], f[1], f[2]);
                 });

  return {boundary_ordered_faces.begin(), boundary_ordered_faces.end()};
}

std::set<SortedTriplet<int>> GetInteriorFaceSet(
    const std::set<SortedTriplet<int>>& boundary_face_set,
    const VolumeMesh<double>& mesh_M) {
  // Four triangle faces of a tetrahedron.
  const int tetrahedron_faces[4][3] = {
      {1, 2, 3},
      {3, 2, 0},
      {2, 1, 0},
      {1, 3, 0}
  };
  std::set<SortedTriplet<int>> interior_face_set;
  for (const VolumeElement& tetrahedron : mesh_M.tetrahedra()) {
    for (const auto& face_vertices : tetrahedron_faces) {
      SortedTriplet<int> face(tetrahedron.vertex(face_vertices[0]),
                              tetrahedron.vertex(face_vertices[1]),
                              tetrahedron.vertex(face_vertices[2]));
      if (1 != boundary_face_set.count(face)) {
        interior_face_set.insert(face);
      }
    }
  }

  return interior_face_set;
}

std::set<SortedPair<int>> GetBoundaryEdgeSet(const VolumeMesh<double>& mesh_M) {
  const std::vector<std::array<int, 3>> boundary_faces =
      IdentifyBoundaryFaces(mesh_M.tetrahedra());
  std::set<SortedPair<int>> boundary_edge_set;
  for (const std::array<int, 3>& f : boundary_faces) {
    boundary_edge_set.insert(SortedPair<int>(f[0], f[1]));
    boundary_edge_set.insert(SortedPair<int>(f[1], f[2]));
    boundary_edge_set.insert(SortedPair<int>(f[2], f[0]));
  }

  return boundary_edge_set;
}

std::set<SortedPair<int>> GetInteriorEdgeSet(
    const std::set<SortedPair<int>>& boundary_edge_set,
    const VolumeMesh<double>& mesh_M) {
  std::set<SortedPair<int>> interior_edge_set;
  const int tetrahedron_edges[6][2] = {{0, 1}, {0, 2}, {0, 3},
                                       {1, 2}, {1, 3}, {2, 3}};
  for (const VolumeElement& tetrahedron : mesh_M.tetrahedra()) {
    for (const auto& edge_vertices : tetrahedron_edges) {
      SortedPair<int> edge(tetrahedron.vertex(edge_vertices[0]),
                           tetrahedron.vertex(edge_vertices[1]));
      if (1 != boundary_edge_set.count(edge)) {
        interior_edge_set.insert(edge);
      }
    }
  }

  return interior_edge_set;
}

}  // namespace

std::vector<int> DetectZeroTetrahedron(const VolumeMesh<double>& mesh_M) {
  const std::unordered_set<int> boundary_vertex_set =
      GetBoundaryVertexSet(mesh_M);

  std::vector<int> null_tetrahedra{};
  for (int tetrahedron_index = 0; tetrahedron_index < mesh_M.num_elements();
       ++tetrahedron_index) {
    const VolumeElement& tetrahedron = mesh_M.element(tetrahedron_index);
    int num_boundary_vertices_of_tetrahedron = 0;
    for (int j = 0; j < mesh_M.kVertexPerElement; ++j) {
      if (boundary_vertex_set.count(tetrahedron.vertex(j)) == 1) {
        ++num_boundary_vertices_of_tetrahedron;
      }
    }
    if (num_boundary_vertices_of_tetrahedron == mesh_M.kVertexPerElement) {
      null_tetrahedra.push_back(tetrahedron_index);
    }
  }
  return null_tetrahedra;
}

std::vector<SortedTriplet<int>> DetectZeroInteriorTriangle(
    const VolumeMesh<double>& mesh_M) {
  const std::unordered_set<int> boundary_vertex_set =
      GetBoundaryVertexSet(mesh_M);
  const std::set<SortedTriplet<int>> boundary_face_set =
      GetBoundaryFaceSet(mesh_M);
  const std::set<SortedTriplet<int>> interior_face_set =
      GetInteriorFaceSet(boundary_face_set, mesh_M);

  std::vector<SortedTriplet<int>> null_interior_triangles;
  for (const SortedTriplet<int>& interior_triangle : interior_face_set) {
    if (1 == boundary_vertex_set.count(interior_triangle.first()) &&
        1 == boundary_vertex_set.count(interior_triangle.second()) &&
        1 == boundary_vertex_set.count(interior_triangle.third())) {
      null_interior_triangles.push_back(interior_triangle);
    }
  }

  return null_interior_triangles;
}

std::vector<SortedPair<int>> DetectZeroInteriorEdge(
    const VolumeMesh<double>& mesh_M) {
  const std::unordered_set<int> boundary_vertex_set =
      GetBoundaryVertexSet(mesh_M);
  const std::set<SortedPair<int>> boundary_edge_set =
      GetBoundaryEdgeSet(mesh_M);
  const std::set<SortedPair<int>> interior_edge_set =
      GetInteriorEdgeSet(boundary_edge_set, mesh_M);

  std::vector<SortedPair<int>> null_interior_edges;
  for (const SortedPair<int>& interior_edge : interior_edge_set) {
    if (1 == boundary_vertex_set.count(interior_edge.first()) &&
        1 == boundary_vertex_set.count(interior_edge.second())) {
      null_interior_edges.push_back(interior_edge);
    }
  }
  return null_interior_edges;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
