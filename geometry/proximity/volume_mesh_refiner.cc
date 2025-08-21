#include "drake/geometry/proximity/volume_mesh_refiner.h"

#include <array>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/detect_zero_simplex.h"
#include "drake/geometry/proximity/mesh_to_vtk.h"
#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

void VolumeMeshRefiner::ResetVertexToTetrahedra(void) {
  vertex_to_tetrahedra_.clear();
  // Initialize each entry as an empty list.
  vertex_to_tetrahedra_.resize(vertices_.size(), {});
  int num_tetrahedra = tetrahedra_.size();
  for (int i = 0; i < num_tetrahedra; ++i) {
    for (int j = 0; j < 4; ++j) {
      vertex_to_tetrahedra_[tetrahedra_[i].vertex(j)].push_back(i);
    }
  }
}

VolumeMesh<double> VolumeMeshRefiner::Refine() {
  tetrahedra_ = input_mesh_.tetrahedra();
  vertices_ = input_mesh_.vertices();
  ResetVertexToTetrahedra();

  // I found that refining the mesh in this order:
  //   1. problematic edges,
  //   2. then problematic triangles,
  //   3. then problematic tetrahedra
  // gives more economical output than the opposite order:
  //   1. problematic tetrahedra,
  //   2. then problematic triangles,
  //   3. then problematic edges
  // because refining problematic edges has a chance to eliminate some
  // problematic triangles and tetrahedra indirectly.  On the other hand,
  // refining problematic tetrahedra does not eliminate problematic triangles
  // and edges.

  // TODO(DamrongGuoy) After changing Detect*WithAllBoundaryVertices() in
  //  detect_zero_simplex.h to take a `const std::vector<VolumeElement>&`
  //  instead of `const VolumeMesh&`, avoid the copy constructor of
  //  VolumeMesh in the code below. The code will change from calling
  //  copy constructor:
  //  for (...DetectInteriorEdgeWithAllBoundaryVertices(
  //              // VolumeMesh copy constructor
  //              {std::vector<VolumeElement>(tetrahedra_),
  //              std::vector<Vector3d>(vertices_)}))
  //  to the more efficient:
  //  for (...DetectInteriorEdgeWithAllBoundaryVertices(tetrahedra_))

  for (const SortedPair<int>& edge : DetectInteriorEdgeWithAllBoundaryVertices(
           {std::vector<VolumeElement>(tetrahedra_),
            std::vector<Vector3d>(vertices_)})) {
    RefineEdge(edge);
  }
  for (const SortedTriplet<int>& triangle :
       DetectInteriorTriangleWithAllBoundaryVertices(
           {std::vector<VolumeElement>(tetrahedra_),
            std::vector<Vector3d>(vertices_)})) {
    RefineTriangle(triangle);
  }
  for (const int tetrahedron : DetectTetrahedronWithAllBoundaryVertices(
           {std::vector<VolumeElement>(tetrahedra_),
            std::vector<Vector3d>(vertices_)})) {
    RefineTetrahedron(tetrahedron);
  }

  return {std::vector<VolumeElement>(tetrahedra_),
          std::vector<Vector3d>(vertices_)};
}

void VolumeMeshRefiner::RefineTetrahedron(int tetrahedron) {
  const int v0 = tetrahedra_.at(tetrahedron).vertex(0);
  const int v1 = tetrahedra_.at(tetrahedron).vertex(1);
  const int v2 = tetrahedra_.at(tetrahedron).vertex(2);
  const int v3 = tetrahedra_.at(tetrahedron).vertex(3);

  vertices_.emplace_back((vertices_.at(v0) + vertices_.at(v1) +
                          vertices_.at(v2) + vertices_.at(v3)) /
                         4);
  const int new_vertex = vertices_.size() - 1;
  DRAKE_THROW_UNLESS(vertices_.size() == vertex_to_tetrahedra_.size() + 1);
  vertex_to_tetrahedra_.push_back({});

  CutTetrahedron(tetrahedron, {v0, v1, v2, v3}, new_vertex);
}

void VolumeMeshRefiner::RefineTriangle(const SortedTriplet<int>& triangle) {
  const int v0 = triangle.first();
  const int v1 = triangle.second();
  const int v2 = triangle.third();

  vertices_.emplace_back(
      (vertices_.at(v0) + vertices_.at(v1) + vertices_.at(v2)) / 3);
  const int new_vertex = vertices_.size() - 1;
  DRAKE_THROW_UNLESS(vertices_.size() == vertex_to_tetrahedra_.size() + 1);
  vertex_to_tetrahedra_.push_back({});

  std::vector<int> incident_tetrahedra = GetTetrahedraOnTriangle(v0, v1, v2);
  DRAKE_THROW_UNLESS(incident_tetrahedra.size() == 2);
  CutTetrahedron(incident_tetrahedra[0], {v0, v1, v2}, new_vertex);
  CutTetrahedron(incident_tetrahedra[1], {v0, v1, v2}, new_vertex);
}

void VolumeMeshRefiner::RefineEdge(const SortedPair<int>& edge) {
  const int v0 = edge.first();
  const int v1 = edge.second();

  vertices_.emplace_back((vertices_.at(v0) + vertices_.at(v1)) / 2);
  const int new_vertex = vertices_.size() - 1;
  DRAKE_THROW_UNLESS(vertices_.size() == vertex_to_tetrahedra_.size() + 1);
  vertex_to_tetrahedra_.push_back({});

  std::vector<int> incident_tetrahedra = GetTetrahedraOnEdge(v0, v1);
  DRAKE_THROW_UNLESS(incident_tetrahedra.size() > 0);
  for (const int tetrahedron : incident_tetrahedra) {
    CutTetrahedron(tetrahedron, {v0, v1}, new_vertex);
  }
}

void VolumeMeshRefiner::CutTetrahedron(const int tetrahedron,
                                       const std::vector<int>& sub_simplex,
                                       const int new_vertex) {
  int num_tetrahedra = tetrahedra_.size();
  DRAKE_THROW_UNLESS(0 <= tetrahedron && tetrahedron < num_tetrahedra);
  const std::array<int, 4> original{
      tetrahedra_[tetrahedron].vertex(0), tetrahedra_[tetrahedron].vertex(1),
      tetrahedra_[tetrahedron].vertex(2), tetrahedra_[tetrahedron].vertex(3)};
  int num_vertices = vertices_.size();
  DRAKE_THROW_UNLESS(0 <= new_vertex && new_vertex < num_vertices);
  // Example. The original tetrahedron has vertices v0,v1,v2,v3. Vertex u is
  // the new vertex. If the sub-simplex is the edge v1,v2, the two new
  // tetrahedra are:
  //     v0, u,v2,v3
  //     v0,v1, u,v3.
  // If the sub-simplex is the triangle v1,v2,v3, the three new tetrahedra are:
  //     v0, u,v2,v3
  //     v0,v1, u,v3
  //     v0,v1,v2, u.
  // If the sub-simplex is the tetrahedron v0,v1,v2,v3 itself, the four new
  // tetrahedra are:
  //      u,v1,v2,v3
  //     v0, u,v2,v3
  //     v0,v1, u,v3
  //     v0,v1,v2, u.
  int num_sub_simplex_vertices = sub_simplex.size();
  DRAKE_THROW_UNLESS(num_sub_simplex_vertices >= 2);
  DRAKE_THROW_UNLESS(num_sub_simplex_vertices <= 4);
  for (int i = 0; i < num_sub_simplex_vertices; ++i) {
    const int local_index = std::distance(
        original.begin(),
        std::find(original.begin(), original.end(), sub_simplex[i]));

    // Confirm that sub_simplex[i] is a vertex of the original tetrahedron.
    DRAKE_THROW_UNLESS(0 <= local_index && local_index < 4);

    std::array<int, 4> replacement = original;
    replacement[local_index] = new_vertex;
    if (i == 0) {
      std::erase(vertex_to_tetrahedra_[original[local_index]], tetrahedron);
      tetrahedra_[tetrahedron] = VolumeElement(replacement.data());
      vertex_to_tetrahedra_[new_vertex].push_back(tetrahedron);
    } else {
      tetrahedra_.emplace_back(replacement.data());
      const int new_tetrahedron = tetrahedra_.size() - 1;
      for (int v : replacement) {
        vertex_to_tetrahedra_[v].push_back(new_tetrahedron);
      }
    }
  }
}

std::vector<int> VolumeMeshRefiner::GetTetrahedraOnTriangle(int v0, int v1,
                                                            int v2) const {
  DRAKE_THROW_UNLESS(v0 != v1 && v1 != v2 && v2 != v0);
  std::vector<int> incident_tetrahedra;
  const std::vector<int>& tetrahedra0 = vertex_to_tetrahedra_[v0];
  const std::vector<int>& tetrahedra1 = vertex_to_tetrahedra_[v1];
  const std::vector<int>& tetrahedra2 = vertex_to_tetrahedra_[v2];
  for (const int& tet0 : tetrahedra0) {
    if (std::find(tetrahedra1.begin(), tetrahedra1.end(), tet0) !=
        tetrahedra1.end()) {
      if (std::find(tetrahedra2.begin(), tetrahedra2.end(), tet0) !=
          tetrahedra2.end()) {
        incident_tetrahedra.push_back(tet0);
      }
    }
  }
  return incident_tetrahedra;
}

std::vector<int> VolumeMeshRefiner::GetTetrahedraOnEdge(int v0, int v1) const {
  DRAKE_THROW_UNLESS(v0 != v1);
  std::vector<int> incident_tetrahedra;
  const std::vector<int>& tetrahedra0 = vertex_to_tetrahedra_[v0];
  const std::vector<int>& tetrahedra1 = vertex_to_tetrahedra_[v1];
  for (const int& tet0 : tetrahedra0) {
    if (std::find(tetrahedra1.begin(), tetrahedra1.end(), tet0) !=
        tetrahedra1.end()) {
      incident_tetrahedra.push_back(tet0);
    }
  }
  return incident_tetrahedra;
}

}  // namespace internal

VolumeMesh<double> RefineVolumeMesh(const VolumeMesh<double>& mesh) {
  // Check for problematic simplices.
  std::vector<int> bad_tets =
      internal::DetectTetrahedronWithAllBoundaryVertices(mesh);
  std::vector<internal::SortedTriplet<int>> bad_triangles =
      internal::DetectInteriorTriangleWithAllBoundaryVertices(mesh);
  std::vector<SortedPair<int>> bad_edges =
      internal::DetectInteriorEdgeWithAllBoundaryVertices(mesh);

  // If no problems found, return the input mesh.
  if (bad_tets.empty() && bad_triangles.empty() && bad_edges.empty()) {
    return mesh;
  }

  // Refine the mesh.
  return internal::VolumeMeshRefiner(mesh).Refine();
}

std::string RefineVolumeMeshIntoVtkFileContents(const MeshSource& mesh_source) {
  const VolumeMesh<double> original =
      internal::ReadVtkToVolumeMesh(mesh_source);
  const VolumeMesh<double> refined = RefineVolumeMesh(original);
  return internal::WriteVolumeMeshToVtkFileContents(
      refined, "refined by //geometry/proximity:volume_mesh_refiner");
}

}  // namespace geometry
}  // namespace drake
