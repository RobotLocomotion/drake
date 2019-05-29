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
std::pair<VolumeMesh<T>, std::vector<bool>>
SphereMeshGenerator<T>::MakeSphereMeshLevel0() {
  std::vector<VolumeElement> tetrahedra;
  std::vector<VolumeVertex<T>> vertices;

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
  // pointing towards the outside. The fourth vertex is on the "negative" side
  // of the plane defined by this normal.

  using V = VolumeVertexIndex;

  // Top tetrahedra.
  tetrahedra.emplace_back(V(0), V(2), V(1), V(5));
  tetrahedra.emplace_back(V(0), V(3), V(2), V(5));
  tetrahedra.emplace_back(V(0), V(4), V(3), V(5));
  tetrahedra.emplace_back(V(0), V(1), V(4), V(5));

  // Bottom tetrahedra.
  tetrahedra.emplace_back(V(0), V(1), V(2), V(6));
  tetrahedra.emplace_back(V(0), V(2), V(3), V(6));
  tetrahedra.emplace_back(V(0), V(3), V(4), V(6));
  tetrahedra.emplace_back(V(0), V(4), V(1), V(6));

  // Indicate what vertices are on the surface of the sphere.
  // All vertices are boundaries but the first one at (0, 0, 0).
  std::vector<bool> is_boundary(7, true);
  is_boundary[0] = false;

  return std::make_pair(
      VolumeMesh<T>(std::move(tetrahedra), std::move(vertices)), is_boundary);
}

template <typename T>
std::pair<VolumeMesh<T>, std::vector<bool>>
SphereMeshGenerator<T>::RefineTetrahdron(const Vector3<T>& A,
                                         const Vector3<T>& B,
                                         const Vector3<T>& C,
                                         const Vector3<T>& D,
                                         const std::vector<bool>& is_boundary) {
  // Aliases to the indexes, just to ease writing the code.
  // That is, index a corresponds to vertex A, index b to vertex B, etc.
  const VolumeVertexIndex a(0);
  const VolumeVertexIndex b(1);
  const VolumeVertexIndex c(2);
  const VolumeVertexIndex d(3);
  const VolumeVertexIndex e(4);
  const VolumeVertexIndex f(5);
  const VolumeVertexIndex g(6);
  const VolumeVertexIndex h(7);
  const VolumeVertexIndex i(8);
  const VolumeVertexIndex j(9);

  // Each tetrahedra is split into 8 sub-tetrahedra.
  std::vector<VolumeElement> tetrahedra;
  std::vector<VolumeVertex<T>> vertices;  // Original 4 plus 6, one per edge.
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
  // Currently we arbitrarily choose to partition along the GH edge.
  auto SplitAlongGH = [&]() {
    std::vector<VolumeElement> inner_tets;
    inner_tets.emplace_back(g, h, i, e);
    inner_tets.emplace_back(g, f, h, e);
    inner_tets.emplace_back(g, i, h, j);
    inner_tets.emplace_back(g, h, f, j);
    return inner_tets;
  };

  // Split the internal octahedron EFGHIJ into four sub-tetrahedra.
  auto inner_tets = SplitAlongGH();
  std::copy(inner_tets.begin(), inner_tets.end(), back_inserter(tetrahedra));

  return std::make_pair(
      VolumeMesh<T>(std::move(tetrahedra), std::move(vertices)),
      split_vertex_is_boundary);
}

template <typename T>
std::pair<VolumeMesh<T>, std::vector<bool>> SphereMeshGenerator<T>::RefineMesh(
    const VolumeMesh<T>& mesh, const std::vector<bool>& is_boundary) {
  std::vector<VolumeElement> split_mesh_tetrahedra;
  std::vector<VolumeVertex<T>> split_mesh_vertices;
  std::vector<bool> split_is_boundary;
  for (const auto& t : mesh.tetrahedra()) {
    const auto& A = mesh.vertex(t.vertex(0)).r_MV();
    const auto& B = mesh.vertex(t.vertex(1)).r_MV();
    const auto& C = mesh.vertex(t.vertex(2)).r_MV();
    const auto& D = mesh.vertex(t.vertex(3)).r_MV();

    // Vector that flags A, B, C and D as boundary vertices or not.
    std::vector<bool> tet_is_boundary;
    tet_is_boundary.push_back(is_boundary[t.vertex(0)]);
    tet_is_boundary.push_back(is_boundary[t.vertex(1)]);
    tet_is_boundary.push_back(is_boundary[t.vertex(2)]);
    tet_is_boundary.push_back(is_boundary[t.vertex(3)]);

    const std::pair<VolumeMesh<T>, std::vector<bool>> split_pair =
        RefineTetrahdron(A, B, C, D, tet_is_boundary);
    const VolumeMesh<T>& split_tet = split_pair.first;
    const std::vector<bool>& split_tet_is_boundary = split_pair.second;
    const int num_vertices = static_cast<int>(split_mesh_vertices.size());

    std::copy(split_tet.vertices().begin(), split_tet.vertices().end(),
              back_inserter(split_mesh_vertices));
    std::copy(split_tet_is_boundary.begin(), split_tet_is_boundary.end(),
              back_inserter(split_is_boundary));

    // Add the new tets. Offset the vertex indexes to account for the indexing
    // within the full mesh.
    std::transform(split_tet.tetrahedra().begin(), split_tet.tetrahedra().end(),
                   back_inserter(split_mesh_tetrahedra),
                   [num_vertices](const VolumeElement& tet) {
                     using V = VolumeVertexIndex;
                     return VolumeElement(V(tet.vertex(0) + num_vertices),
                                          V(tet.vertex(1) + num_vertices),
                                          V(tet.vertex(2) + num_vertices),
                                          V(tet.vertex(3) + num_vertices));
                   });
  }

  return std::make_pair(VolumeMesh<T>(std::move(split_mesh_tetrahedra),
                                      std::move(split_mesh_vertices)),
                        split_is_boundary);
}

template <typename T>
VolumeMesh<T> SphereMeshGenerator<T>::MakeSphereMesh(int refinement_level) {
  DRAKE_DEMAND(refinement_level >= 0);
  std::pair<VolumeMesh<T>, std::vector<bool>> pair = MakeSphereMeshLevel0();
  VolumeMesh<T>& mesh = pair.first;
  std::vector<bool>& is_boundary = pair.second;

  for (int level = 1; level <= refinement_level; ++level) {
    auto split_pair = RefineMesh(mesh, is_boundary);
    mesh = split_pair.first;
    is_boundary = split_pair.second;
    DRAKE_DEMAND(mesh.vertices().size() == is_boundary.size());
  }

  return std::move(mesh);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::geometry::internal::SphereMeshGenerator)
