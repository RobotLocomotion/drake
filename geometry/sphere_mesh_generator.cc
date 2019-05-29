#include "drake/geometry/sphere_mesh_generator.h"

#include <algorithm>
#include <iterator>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
TetrahedraMesh<T> SphereMeshGenerator<T>::MakeSphereMeshLevel0() {
  std::vector<Vector4<int>> tetrahedra;
  std::vector<Vector3<T>> vertices;

  // Level "0" consists of the octahedron with vertices on the surface of the
  // a sphere of unit radius.
  vertices.emplace_back(0.0, 0.0, 0.0);
  vertices.emplace_back(1.0, 0.0, 0.0);
  vertices.emplace_back(0.0, 1.0, 0.0);
  vertices.emplace_back(-1.0, 0.0, 0.0);
  vertices.emplace_back(0.0, -1.0, 0.0);
  vertices.emplace_back(0.0, 0.0, 1.0);
  vertices.emplace_back(0.0, 0.0, -1.0);

  // Create tetrahedra. The convention is that the first three vertices define
  // the "base" of the tetrahedron with its right-handed normal vector
  // pointing towards the outside. The four vertex is therefore on the
  // "negative" side of the plane defined by this normal.

  // Top tetrahedra.
  tetrahedra.emplace_back(0, 2, 1, 5);
  tetrahedra.emplace_back(0, 3, 2, 5);
  tetrahedra.emplace_back(0, 4, 3, 5);
  tetrahedra.emplace_back(0, 1, 4, 5);

  // Bottom tetrahedra.
  tetrahedra.emplace_back(0, 1, 2, 6);
  tetrahedra.emplace_back(0, 2, 3, 6);
  tetrahedra.emplace_back(0, 3, 4, 6);
  tetrahedra.emplace_back(0, 4, 1, 6);

  return TetrahedraMesh<T>{vertices, tetrahedra};
}

template <typename T>
std::pair<TetrahedraMesh<T>, std::vector<bool>>
SphereMeshGenerator<T>::SplitTetrahedra(const Vector3<T>& A,
                                        const Vector3<T>& B,
                                        const Vector3<T>& C,
                                        const Vector3<T>& D,
                                        const std::vector<bool>& is_boundary) {
  // Aliases to the indexes, just to ease writting the code.
  const int a = 0;
  const int b = 1;
  const int c = 2;
  const int d = 3;
  const int e = 4;
  const int f = 5;
  const int g = 6;
  const int h = 7;
  const int i = 8;
  const int j = 9;

  // Each tetrahedra is split into 8 sug-tetrahedra.
  std::vector<Vector4<int>> tetrahedra;
  std::vector<Vector3<T>> vertices;  // Original 4 plus 6, one per edge.
  Vector3<T> E = (A + B) / 2.0;
  Vector3<T> F = (A + C) / 2.0;
  Vector3<T> G = (A + D) / 2.0;
  Vector3<T> H = (B + C) / 2.0;
  Vector3<T> I = (B + D) / 2.0;
  Vector3<T> J = (C + D) / 2.0;

  // Mark which vertices belong to the boundary.
  std::vector<bool> split_vertex_is_boundary(10);
  split_vertex_is_boundary[a] = is_boundary[a];
  split_vertex_is_boundary[b] = is_boundary[b];
  split_vertex_is_boundary[c] = is_boundary[c];
  split_vertex_is_boundary[d] = is_boundary[d];
  split_vertex_is_boundary[e] = is_boundary[a] && is_boundary[b];
  split_vertex_is_boundary[f] = is_boundary[a] && is_boundary[c];
  split_vertex_is_boundary[g] = is_boundary[a] && is_boundary[d];
  split_vertex_is_boundary[h] = is_boundary[b] && is_boundary[c];
  split_vertex_is_boundary[i] = is_boundary[b] && is_boundary[d];
  split_vertex_is_boundary[j] = is_boundary[c] && is_boundary[d];

  //  Project boundary vertices onto the surface of the sphere.
  if (split_vertex_is_boundary[e]) E.normalize();
  if (split_vertex_is_boundary[f]) F.normalize();
  if (split_vertex_is_boundary[g]) G.normalize();
  if (split_vertex_is_boundary[h]) H.normalize();
  if (split_vertex_is_boundary[i]) I.normalize();
  if (split_vertex_is_boundary[j]) J.normalize();

  // Place the new vertices in the vector of vertices.
  vertices.emplace_back(A);
  vertices.emplace_back(B);
  vertices.emplace_back(C);
  vertices.emplace_back(D);
  vertices.emplace_back(E);
  vertices.emplace_back(F);
  vertices.emplace_back(G);
  vertices.emplace_back(H);
  vertices.emplace_back(I);
  vertices.emplace_back(J);

  // The four tetrahedra at the corners.
  tetrahedra.emplace_back(a, e, f, g);
  tetrahedra.emplace_back(b, h, e, i);
  tetrahedra.emplace_back(f, h, c, j);
  tetrahedra.emplace_back(j, g, i, d);

  // TODO(amcastro-tri): Split along EJ, FI and GH and choose the partition with
  // best quality factor as described in [Everett, 1997], see the class's
  // documentation.
  auto SplitAlongGH = [&]() {
    std::vector<Vector4<int>> inner_tets;
    inner_tets.emplace_back(g, h, i, e);
    inner_tets.emplace_back(g, f, h, e);
    inner_tets.emplace_back(g, i, h, j);
    inner_tets.emplace_back(g, h, f, j);
    return inner_tets;
  };

  // Split the internal octahedron EFGHIJ into four sub-tetrahedra.
  auto inner_tets = SplitAlongGH();
  std::copy(inner_tets.begin(), inner_tets.end(), back_inserter(tetrahedra));

  return std::make_pair(TetrahedraMesh<T>{vertices, tetrahedra},
                        split_vertex_is_boundary);
}

template <typename T>
std::pair<TetrahedraMesh<T>, std::vector<bool>>
SphereMeshGenerator<T>::SplitMesh(const TetrahedraMesh<T>& mesh,
                                  const std::vector<bool>& is_boundary) {
  TetrahedraMesh<T> split_mesh;
  std::vector<bool> split_is_boundary;
  for (const auto& t : mesh.tetrahedra) {
    const auto& A = mesh.vertices[t[0]];
    const auto& B = mesh.vertices[t[1]];
    const auto& C = mesh.vertices[t[2]];
    const auto& D = mesh.vertices[t[3]];

    // Vector that flags A, B, C and D as boundary vertices or not.
    std::vector<bool> tet_is_boundary;
    tet_is_boundary.push_back(is_boundary[t[0]]);
    tet_is_boundary.push_back(is_boundary[t[1]]);
    tet_is_boundary.push_back(is_boundary[t[2]]);
    tet_is_boundary.push_back(is_boundary[t[3]]);

    const auto split_pair = SplitTetrahedra(A, B, C, D, tet_is_boundary);
    const auto& split_tet = split_pair.first;
    const auto& split_tet_is_boundary = split_pair.second;
    const int num_vertices = split_mesh.vertices.size();

    std::copy(split_tet.vertices.begin(), split_tet.vertices.end(),
              back_inserter(split_mesh.vertices));
    std::copy(split_tet_is_boundary.begin(), split_tet_is_boundary.end(),
              back_inserter(split_is_boundary));

    // Add the new tests. Offset the vertex indexes to account for the indexing
    // within the full mesh.
    std::transform(split_tet.tetrahedra.begin(), split_tet.tetrahedra.end(),
                   back_inserter(split_mesh.tetrahedra),
                   [num_vertices](const Vector4<int>& tet) {
                     return tet + Vector4<int>::Ones() * num_vertices;
                   });
  }

  return std::make_pair(split_mesh, split_is_boundary);
}

template <typename T>
TetrahedraMesh<T> SphereMeshGenerator<T>::MakeSphereMesh(int refinement_level) {
  TetrahedraMesh<T> mesh = MakeSphereMeshLevel0();
  // All vertices are boundaries but the first one at (0, 0, 0).
  std::vector<bool> is_boundary(7, true);
  is_boundary[0] = false;

  for (int level = 1; level <= refinement_level; ++level) {
    auto split_pair = SplitMesh(mesh, is_boundary);
    mesh = split_pair.first;
    is_boundary = split_pair.second;
    DRAKE_DEMAND(mesh.vertices.size() == is_boundary.size());
  }

  return mesh;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::geometry::internal::TetrahedraMesh)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::geometry::internal::SphereMeshGenerator)
