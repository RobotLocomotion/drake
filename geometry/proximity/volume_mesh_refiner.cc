#include "drake/geometry/proximity/volume_mesh_refiner.h"

#include <array>
#include <set>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/detect_null_simplex.h"
#include "drake/geometry/proximity/sorted_triplet.h"

namespace drake {
namespace geometry {
namespace internal {

VolumeMeshRefiner::VolumeMeshRefiner(const VolumeMesh<double>& input_mesh):
  input_mesh_(input_mesh) {
}

VolumeMesh<double> VolumeMeshRefiner::refine() {
  tetrahedra_ = input_mesh_.tetrahedra();
  vertices_ = input_mesh_.vertices();

  for (const SortedPair<int>& edge : DetectNullInteriorEdge(input_mesh_)) {
    RefineEdge(edge);
  }
  VolumeMesh<double> refined_edges = CreateMesh(tetrahedra_, vertices_);

  for (const SortedTriplet<int>& triangle :
      DetectNullInteriorTriangle(refined_edges)) {
    RefineTriangle(triangle);
  }
  VolumeMesh<double> refined_triangles = CreateMesh(tetrahedra_, vertices_);

  for (const int tetrahedron : DetectNullTetrahedron(refined_triangles)) {
    RefineTetrahedron(tetrahedron);
  }
  VolumeMesh<double> refined_tetrahedra = CreateMesh(tetrahedra_, vertices_);

  return refined_tetrahedra;
}

void VolumeMeshRefiner::RefineTetrahedron(int tetrahedron) {
  const int v0 = tetrahedra_.at(tetrahedron).vertex(0);
  const int v1 = tetrahedra_.at(tetrahedron).vertex(1);
  const int v2 = tetrahedra_.at(tetrahedron).vertex(2);
  const int v3 = tetrahedra_.at(tetrahedron).vertex(3);

  Vector3<double> new_point = (vertices_.at(v0) + vertices_.at(v1) +
                               vertices_.at(v2) + vertices_.at(v3)) /
                              4;
  vertices_.emplace_back(new_point);

  const int v4 = vertices_.size() - 1;

  ReplaceOneTetrahedronWithFour(tetrahedron, v4);
}

void VolumeMeshRefiner::RefineTriangle(const SortedTriplet<int>& triangle) {
  const int v0 = triangle.first();
  const int v1 = triangle.second();
  const int v2 = triangle.third();

  std::vector<int> incident_tetrahedra =
      GetTetrahedraOnTriangle(v0, v1, v2);
  DRAKE_DEMAND(incident_tetrahedra.size() == 2);

  vertices_.emplace_back(
      (vertices_.at(v0) + vertices_.at(v1) + vertices_.at(v2)) / 3);
  const int new_vertex = vertices_.size() - 1;

  ReplaceOneTetrahedronWithThree(incident_tetrahedra[0], triangle, new_vertex);
  ReplaceOneTetrahedronWithThree(incident_tetrahedra[1], triangle, new_vertex);
}

void VolumeMeshRefiner::RefineEdge(const SortedPair<int>& edge) {
  const int v0 = edge.first();
  const int v1 = edge.second();

  std::vector<int> incident_tetrahedra = GetTetrahedraOnEdge(v0, v1);
  DRAKE_DEMAND(incident_tetrahedra.size() > 0);

  vertices_.emplace_back((vertices_.at(v0) + vertices_.at(v1)) / 2);
  const int new_vertex = vertices_.size() - 1;

  for (const int tetrahedron : incident_tetrahedra) {
    ReplaceOneTetrahedronWithTwo(tetrahedron, edge, new_vertex);
  }
}

void VolumeMeshRefiner::ReplaceOneTetrahedronWithFour(int tetrahedron,
                                                      int new_vertex) {
  const std::array<int, 4> v{tetrahedra_.at(tetrahedron).vertex(0),
                             tetrahedra_.at(tetrahedron).vertex(1),
                             tetrahedra_.at(tetrahedron).vertex(2),
                             tetrahedra_.at(tetrahedron).vertex(3)};
  std::array<int, 4> x = v;
  x[0] = new_vertex;
  tetrahedra_.at(tetrahedron) = VolumeElement(x[0], x[1], x[2], x[3]);
  for (int i = 1; i < 4; ++i) {
    x = v;
    x[i] = new_vertex;
    tetrahedra_.emplace_back(x[0], x[1], x[2], x[3]);
  }
}

void VolumeMeshRefiner::ReplaceOneTetrahedronWithThree(
    int tetrahedron, const SortedTriplet<int>& triangle, int new_vertex) {
  // tetrahedron = v[0] v[1] v[2] v[3]
  const std::array<int, 4> v{tetrahedra_.at(tetrahedron).vertex(0),
                             tetrahedra_.at(tetrahedron).vertex(1),
                             tetrahedra_.at(tetrahedron).vertex(2),
                             tetrahedra_.at(tetrahedron).vertex(3)};

  const int i0 = std::find(v.begin(), v.end(), triangle.first()) - v.begin();
  const int i1 = std::find(v.begin(), v.end(), triangle.second()) - v.begin();
  const int i2 = std::find(v.begin(), v.end(), triangle.third()) - v.begin();
  DRAKE_DEMAND(i0 < 4);
  DRAKE_DEMAND(i1 < 4);
  DRAKE_DEMAND(i2 < 4);

  // tet_0 = replace v[i0] by new_vertex
  // tet_1 = replace v[i1] by new_vertex
  // tet_2 = replace v[i2] by new_vertex
  std::array<int, 4> x = v;
  x[i0] = new_vertex;
  tetrahedra_.at(tetrahedron) = VolumeElement(x[0], x[1], x[2], x[3]);
  for (int j : {i1, i2}) {
    x = v;
    x[j] = new_vertex;
    tetrahedra_.emplace_back(x[0], x[1], x[2], x[3]);
  }
}

void VolumeMeshRefiner::ReplaceOneTetrahedronWithTwo(
    int tetrahedron, const SortedPair<int>& edge, int new_vertex) {
  // tetrahedron = v[0] v[1] v[2] v[3]
  const std::array<int, 4> v{tetrahedra_.at(tetrahedron).vertex(0),
                             tetrahedra_.at(tetrahedron).vertex(1),
                             tetrahedra_.at(tetrahedron).vertex(2),
                             tetrahedra_.at(tetrahedron).vertex(3)};

  const int i0 = std::find(v.begin(), v.end(), edge.first()) - v.begin();
  const int i1 = std::find(v.begin(), v.end(), edge.second()) - v.begin();
  DRAKE_DEMAND(i0 < 4);
  DRAKE_DEMAND(i1 < 4);

  // tet_0 = replace v[i0] by new_vertex
  // tet_1 = replace v[i1] by new_vertex
  std::array<int, 4> x = v;
  x[i0] = new_vertex;
  tetrahedra_.at(tetrahedron) = VolumeElement(x[0], x[1], x[2], x[3]);
  x = v;
  x[i1] = new_vertex;
  tetrahedra_.emplace_back(x[0], x[1], x[2], x[3]);
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

std::vector<int> VolumeMeshRefiner::GetTetrahedraOnEdge(int v0,
                                                        int v1) const {
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

VolumeMesh<double> VolumeMeshRefiner::CreateMesh(
    const std::vector<VolumeElement>& elements,
    const std::vector<Vector3<double>>& vertices) const {
  std::vector<VolumeElement> elements_copy = elements;
  std::vector<Vector3<double>> vertices_copy = vertices;
  return {std::move(elements_copy), std::move(vertices_copy)};
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
