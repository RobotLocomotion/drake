#include "drake/geometry/proximity/detect_null_simplex.h"

#include <algorithm>
#include <array>
#include <map>
#include <set>
#include <unordered_set>
#include <utility>

#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

std::vector<int> DetectNullTetrahedron(const VolumeMesh<double>& mesh_M) {
  std::vector<int> bv = CollectUniqueVertices(
      IdentifyBoundaryFaces(mesh_M.tetrahedra()));
  const std::unordered_set<int> boundary_vertex_set(bv.begin(), bv.end());

  std::vector<int> zero_tetrahedra{};
  for (int i = 0; i < mesh_M.num_elements(); ++i) {
    const VolumeElement& tetrahedron_i = mesh_M.element(i);
    int num_boundary_vertices_of_tetrahedron_i = 0;
    for (int j = 0; j < mesh_M.kVertexPerElement; ++j) {
      if (boundary_vertex_set.count(tetrahedron_i.vertex(j)) == 1) {
        ++num_boundary_vertices_of_tetrahedron_i;
      }
    }
    if (num_boundary_vertices_of_tetrahedron_i == mesh_M.kVertexPerElement) {
      zero_tetrahedra.push_back(i);
    }
  }
  return zero_tetrahedra;
}

std::vector<SortedTriplet<int>> DetectNullInteriorTriangle(
    const VolumeMesh<double>& mesh_M) {
  const std::vector<std::array<int, 3>> bf =
      IdentifyBoundaryFaces(mesh_M.tetrahedra());
  std::vector<SortedTriplet<int>> bfv;
  for (const std::array<int, 3>& f : bf) {
    bfv.emplace_back(f[0], f[1], f[2]);
  }
  const std::set<SortedTriplet<int>> boundary_triangle_set(
      bfv.begin(), bfv.end());

  const std::vector<int> bv = CollectUniqueVertices(bf);
  const std::unordered_set<int> boundary_vertex_set(bv.begin(), bv.end());

  std::set<SortedTriplet<int>> interior_triangle_set;
  // Four triangle faces of a tetrahedron.
  const int tetrahedron_faces[4][3] = {
      {1, 2, 3},
      {3, 2, 0},
      {2, 1, 0},
      {1, 3, 0}
  };
  for (const VolumeElement& tetrahedron : mesh_M.tetrahedra()) {
    for (const auto& face_vertices : tetrahedron_faces) {
      SortedTriplet<int> triangle(tetrahedron.vertex(face_vertices[0]),
                                  tetrahedron.vertex(face_vertices[1]),
                                  tetrahedron.vertex(face_vertices[2]));
      if (1 != boundary_triangle_set.count(triangle)) {
        interior_triangle_set.insert(triangle);
      }
    }
  }

  std::vector<SortedTriplet<int>> zero_interior_triangles;
  for (const SortedTriplet<int>& interior_triangle : interior_triangle_set) {
    if (1 == boundary_vertex_set.count(interior_triangle.first()) &&
        1 == boundary_vertex_set.count(interior_triangle.second()) &&
        1 == boundary_vertex_set.count(interior_triangle.third())) {
      zero_interior_triangles.push_back(interior_triangle);
    }
  }

  return zero_interior_triangles;
}

std::vector<SortedPair<int>> DetectNullInteriorEdge(
    const VolumeMesh<double>& mesh_M) {
  const std::vector<std::array<int, 3>> bf =
      IdentifyBoundaryFaces(mesh_M.tetrahedra());
  std::set<SortedPair<int>> boundary_edge_set;
  for (const std::array<int, 3>& f : bf) {
    boundary_edge_set.insert(SortedPair<int>(f[0], f[1]));
    boundary_edge_set.insert(SortedPair<int>(f[1], f[2]));
    boundary_edge_set.insert(SortedPair<int>(f[2], f[0]));
  }

  const std::vector<int> bv = CollectUniqueVertices(bf);
  const std::unordered_set<int> boundary_vertex_set(bv.begin(), bv.end());

  std::set<SortedPair<int>> interior_edge_set;
  const int tetrahedron_edges[6][2] = {
      {0, 1},
      {0, 2},
      {0, 3},
      {1, 2},
      {1, 3},
      {2, 3}
  };
  for (const VolumeElement& tetrahedron : mesh_M.tetrahedra()) {
    for (const auto& edge_vertices : tetrahedron_edges) {
      SortedPair<int> edge(tetrahedron.vertex(edge_vertices[0]),
                           tetrahedron.vertex(edge_vertices[1]));
      if (1 != boundary_edge_set.count(edge)) {
        interior_edge_set.insert(edge);
      }
    }
  }

  std::vector<SortedPair<int>> zero_interior_edges;
  for (const SortedPair<int>& interior_edge : interior_edge_set) {
    if (1 == boundary_vertex_set.count(interior_edge.first()) &&
        1 == boundary_vertex_set.count(interior_edge.second())) {
      zero_interior_edges.push_back(interior_edge);
    }
  }
  return zero_interior_edges;
}

VolumeMesh<double> CreateSubMesh(const VolumeMesh<double>& mesh_M,
                                 const std::vector<int>& tetrahedron_indices) {
  std::vector<int> indices(tetrahedron_indices);
  std::sort(indices.begin(), indices.end());
  auto last = std::unique(indices.begin(), indices.end());
  indices.erase(last, indices.end());

  const int num_input_tetrahedra = mesh_M.tetrahedra().size();
  const int num_output_tetrahedra = indices.size();
  DRAKE_DEMAND(num_output_tetrahedra <= num_input_tetrahedra);

  std::vector<VolumeElement> tetrahedra;
  for (int index : indices) {
    DRAKE_DEMAND(0 <= index);
    DRAKE_DEMAND(index < num_input_tetrahedra);
    tetrahedra.push_back(mesh_M.tetrahedra()[index]);
  }
  std::vector<Vector3<double>> vertices = mesh_M.vertices();

  return RemoveUnusedVertices(
      VolumeMesh<double>{std::move(tetrahedra), std::move(vertices)});
}

VolumeMesh<double> RemoveUnusedVertices(const VolumeMesh<double>& mesh_M) {
  // Collect used vertices in arbitrary order.
  std::unordered_set<int> used_vertices;
  const int num_tetrahedra = mesh_M.num_elements();
  for (int i = 0; i < num_tetrahedra; ++i) {
    for (int j = 0; j < 4; ++j) {
      used_vertices.insert(mesh_M.tetrahedra()[i].vertex(j));
    }
  }

  const int num_used_vertices = used_vertices.size();
  const int num_input_vertices = mesh_M.num_vertices();
  if (num_used_vertices == num_input_vertices) {
    return mesh_M;
  }

  // Map between input vertex index and output vertex index in such a way that
  // the vertex ordering is preserved as much as possible.
  std::map<int, int> output_vertex_index;
  std::vector<Vector3<double>> output_vertices;
  output_vertices.reserve(used_vertices.size());
  for (int input = 0; input < num_input_vertices; ++input) {
    if (used_vertices.count(input)) {
      output_vertex_index[input] = output_vertices.size();
      output_vertices.push_back(mesh_M.vertex(input));
    }
  }

  int num_output_vertices = output_vertices.size();
  DRAKE_DEMAND(num_output_vertices == num_used_vertices);

  std::vector<VolumeElement> output_tetrahedra;
  output_tetrahedra.reserve(num_tetrahedra);
  for (const VolumeElement& tetrahedron : mesh_M.tetrahedra()) {
    const int a = output_vertex_index.at(tetrahedron.vertex(0));
    const int b = output_vertex_index.at(tetrahedron.vertex(1));
    const int c = output_vertex_index.at(tetrahedron.vertex(2));
    const int d = output_vertex_index.at(tetrahedron.vertex(3));
    output_tetrahedra.emplace_back(a, b, c, d);
  }

  return {std::move(output_tetrahedra), std::move(output_vertices)};
}

VolumeMesh<double> CutZSubMesh(const VolumeMesh<double>& mesh_M,
                               const double min_z, const double max_z) {
  // Collect tetrahedra that are in the range [min_z, max_z]
  std::vector<int> tetrahedra_in_range;
  const int num_tetrahedra = mesh_M.num_elements();
  for (int i = 0; i < num_tetrahedra; ++i) {
    bool in_range = true;
    for (int j = 0; j < 4; ++j) {
      const int v = mesh_M.tetrahedra()[i].vertex(j);
      if (mesh_M.vertex(v).z() < min_z) {
        in_range = false;
        break;
      }
      if (mesh_M.vertex(v).z() > max_z) {
        in_range = false;
        break;
      }
    }
    if (in_range) {
      tetrahedra_in_range.push_back(i);
    }
  }

  return CreateSubMesh(mesh_M, tetrahedra_in_range);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
