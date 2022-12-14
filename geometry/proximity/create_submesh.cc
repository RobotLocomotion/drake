#include "drake/geometry/proximity/create_submesh.h"

#include <algorithm>
#include <map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace drake {
namespace geometry {
namespace internal {

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

