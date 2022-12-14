#include "drake/geometry/proximity/volume_mesh_refiner.h"

#include <array>
#include <set>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/detect_zero_simplex.h"
#include "drake/geometry/proximity/sorted_triplet.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

VolumeMeshRefiner::VolumeMeshRefiner(const VolumeMesh<double>& input_mesh)
    : input_mesh_(input_mesh) {}

VolumeMesh<double> VolumeMeshRefiner::Refine() {
  tetrahedra_ = input_mesh_.tetrahedra();
  vertices_ = input_mesh_.vertices();

  // I found that refining the mesh in this order:
  //   1. problematic edges,
  //   2. then problematic triangles,
  //   3. then problematic tetrahedra
  // gives more economical output than the opposite order:
  //   A. problematic tetrahedra,
  //   B. then problematic triangles,
  //   C. then problematic edges.
  // because refining problematic edges has a chance to eliminate some
  // problematic triangles and tetrahedra indirectly.  On the other hand,
  // refining problematic tetrahedra does not eliminate problematic triangles
  // and edges.

  for (const SortedPair<int>& edge :
       DetectInteriorEdgeWithAllBoundaryVertices(input_mesh_)) {
    RefineEdge(edge);
  }
  // Checkpoint the lists tetrahedra_ and vertices_ after refining
  // problematic edges because each of the subsequent mesh-refinement
  // operation also increases the lists.
  VolumeMesh<double> refined_edges{std::vector<VolumeElement>(tetrahedra_),
                                   std::vector<Vector3d>(vertices_)};

  for (const SortedTriplet<int>& triangle :
       DetectInteriorTriangleWithAllBoundaryVertices(refined_edges)) {
    RefineTriangle(triangle);
  }
  // Checkpoint the lists tetrahedra_ and vertices_ after refining
  // problematic triangles because each of the subsequent mesh-refinement
  // operation also increases the lists.
  VolumeMesh<double> refined_triangles{std::vector<VolumeElement>(tetrahedra_),
                                       std::vector<Vector3d>(vertices_)};

  for (const int tetrahedron :
       DetectTetrahedronWithAllBoundaryVertices(refined_triangles)) {
    RefineTetrahedron(tetrahedron);
  }
  VolumeMesh<double> refined_tetrahedra{std::vector<VolumeElement>(tetrahedra_),
                                        std::vector<Vector3d>(vertices_)};

  return refined_tetrahedra;
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

  CutTetrahedron(tetrahedron, {v0, v1, v2, v3}, new_vertex);
}

void VolumeMeshRefiner::RefineTriangle(const SortedTriplet<int>& triangle) {
  const int v0 = triangle.first();
  const int v1 = triangle.second();
  const int v2 = triangle.third();

  std::vector<int> incident_tetrahedra = GetTetrahedraOnTriangle(v0, v1, v2);
  DRAKE_DEMAND(incident_tetrahedra.size() == 2);
  vertices_.emplace_back(
      (vertices_.at(v0) + vertices_.at(v1) + vertices_.at(v2)) / 3);
  const int new_vertex = vertices_.size() - 1;

  CutTetrahedron(incident_tetrahedra[0], {v0, v1, v2}, new_vertex);
  CutTetrahedron(incident_tetrahedra[1], {v0, v1, v2}, new_vertex);
}

void VolumeMeshRefiner::RefineEdge(const SortedPair<int>& edge) {
  const int v0 = edge.first();
  const int v1 = edge.second();

  std::vector<int> incident_tetrahedra = GetTetrahedraOnEdge(v0, v1);
  DRAKE_DEMAND(incident_tetrahedra.size() > 0);
  vertices_.emplace_back((vertices_.at(v0) + vertices_.at(v1)) / 2);
  const int new_vertex = vertices_.size() - 1;

  for (const int tetrahedron : incident_tetrahedra) {
    CutTetrahedron(tetrahedron, {v0, v1}, new_vertex);
  }
}

void VolumeMeshRefiner::CutTetrahedron(int tetrahedron,
                                       std::vector<int> sub_simplex,
                                       int new_vertex) {
  int num_sub_simplex_vertices = sub_simplex.size();
  DRAKE_DEMAND(num_sub_simplex_vertices >= 2);
  DRAKE_DEMAND(num_sub_simplex_vertices <= 4);

  const std::array<int, 4> original{tetrahedra_.at(tetrahedron).vertex(0),
                                    tetrahedra_.at(tetrahedron).vertex(1),
                                    tetrahedra_.at(tetrahedron).vertex(2),
                                    tetrahedra_.at(tetrahedron).vertex(3)};

  // Example: Given a sub-simplex v1v2v3 of the original tetrahedron
  // v0v1v2v3 and the new vertex u, we will replace v0v1v2v3 with these two
  // tetrahedra:
  //     v0_u_v2v3
  //     v0v1_u_v3
  //     v0v1v2_u_
  // The three new tetrahedra have their signed volume with the same sign as
  // the original tetrahedron if the new vertex u is inside the sub-simplex.

  int index_0 = std::find(original.begin(), original.end(), sub_simplex[0]) -
                original.begin();
  DRAKE_DEMAND(index_0 >= 0);
  DRAKE_DEMAND(index_0 < 4);

  std::array<int, 4> replacement = original;
  replacement[index_0] = new_vertex;
  tetrahedra_.at(tetrahedron) = VolumeElement(replacement.data());

  for (int i = 1; i < num_sub_simplex_vertices; ++i) {
    int index_i = std::find(original.begin(), original.end(), sub_simplex[i]) -
                  original.begin();
    DRAKE_DEMAND(index_i >= 0);
    DRAKE_DEMAND(index_i < 4);

    replacement = original;
    replacement[index_i] = new_vertex;
    tetrahedra_.emplace_back(replacement.data());
  }
}

std::vector<int> VolumeMeshRefiner::GetTetrahedraOnTriangle(int v0, int v1,
                                                            int v2) const {
  std::vector<int> incident_tetrahedra;
  int num_tetrahedra = tetrahedra_.size();
  for (int tetrahedron = 0; tetrahedron < num_tetrahedra; ++tetrahedron) {
    if (IsTriangleInTetrahedron(v0, v1, v2, tetrahedron)) {
      incident_tetrahedra.push_back(tetrahedron);
    }
  }
  return incident_tetrahedra;
}

std::vector<int> VolumeMeshRefiner::GetTetrahedraOnEdge(int v0, int v1) const {
  std::vector<int> incident_tetrahedra;
  int num_tetrahedra = tetrahedra_.size();
  for (int tetrahedron = 0; tetrahedron < num_tetrahedra; ++tetrahedron) {
    if (IsEdgeInTetrahedron(v0, v1, tetrahedron)) {
      incident_tetrahedra.push_back(tetrahedron);
    }
  }
  return incident_tetrahedra;
}

bool VolumeMeshRefiner::IsTriangleInTetrahedron(int v0, int v1, int v2,
                                                int tetrahedron) const {
  const std::set<int> tetrahedron_vertices{
      tetrahedra_.at(tetrahedron).vertex(0),
      tetrahedra_.at(tetrahedron).vertex(1),
      tetrahedra_.at(tetrahedron).vertex(2),
      tetrahedra_.at(tetrahedron).vertex(3)};
  DRAKE_DEMAND(tetrahedron_vertices.size() == 4);

  return tetrahedron_vertices.count(v0) == 1 &&
         tetrahedron_vertices.count(v1) == 1 &&
         tetrahedron_vertices.count(v2) == 1;
}

bool VolumeMeshRefiner::IsEdgeInTetrahedron(int v0, int v1,
                                            int tetrahedron) const {
  const std::set<int> tetrahedron_vertices{
      tetrahedra_.at(tetrahedron).vertex(0),
      tetrahedra_.at(tetrahedron).vertex(1),
      tetrahedra_.at(tetrahedron).vertex(2),
      tetrahedra_.at(tetrahedron).vertex(3)};
  DRAKE_DEMAND(tetrahedron_vertices.size() == 4);

  return tetrahedron_vertices.count(v0) == 1 &&
         tetrahedron_vertices.count(v1) == 1;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
