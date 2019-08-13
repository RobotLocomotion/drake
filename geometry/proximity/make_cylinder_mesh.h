#pragma once

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

// Helper methods for MakeCylinderMesh().
#ifndef DRAKE_DOXYGEN_CXX

// Determines the boundary type of a vertex of the cylinder mesh
typedef enum {
  VERTEX_INTERNAL,
  VERTEX_CAP,
  VERTEX_SIDE
} CylinderVertexType;

// Project the point p to the side of the cylinder in the XY direction
template<typename T>
Vector3<T> ProjectOntoCylinderSide(const Vector3<T> &p, const double radius) {
  Vector3<T> p_proj = Vector3<T>(p[0], p[1], 0.0);
  auto len = p_proj.norm();
  p_proj.normalize();
  return p + (radius - len) * p_proj;
}

// Project midpoint of two cap vertices so that it's length from the medial
// axis is the mean of the two vertices lengths from the medial axis.
template<typename T>
Vector3<T> ProjectOntoCylinderCap(const Vector3<T> &X, const Vector3<T> &Y) {
  Vector3<T> V = (X + Y) / 2.0;

  Vector3<T> v_proj = Vector3<T>(V[0], V[1], 0.0);
  Vector3<T> x_proj = Vector3<T>(X[0], X[1], 0.0);
  Vector3<T> y_proj = Vector3<T>(Y[0], Y[1], 0.0);

  auto length = v_proj.norm();
  auto desired_length = (x_proj.norm() + y_proj.norm()) / 2.0;

  v_proj.normalize();

  return V + (desired_length - length) * v_proj;
}

// Determine the correct projection based on the boundary type of the vertex
template<typename T>
Vector3<T> ChooseProjection(const Vector3<T> &X,
                            const Vector3<T> &Y,
                            CylinderVertexType v_type,
                            const double radius,
                            const double height) {
  // Project boundary vertices onto the surface of the cylinder.
  const Vector3<T> V = (X + Y) / 2.0;

  // Internal vertices can be projected in the same manner as cap vertices
  // for better quality
  switch (v_type) {
    case VERTEX_SIDE:return ProjectOntoCylinderSide(V, radius);
    case VERTEX_CAP:
    case VERTEX_INTERNAL:return ProjectOntoCylinderCap(X, Y);
    default:return V;
  }
}

// Commutative hash and equals functor for a pair of vertex element indices
// Could be a better hash function, but this particular mixing works
// relatively well for working size meshes
struct VertexPairHashFunction {
  std::size_t operator()(const std::pair<VolumeVertexIndex,
                                         VolumeVertexIndex> &element) const {
    // Take the convention that x is the smaller of the pair
    // so that the hash is commutative
    std::size_t x = std::min(element.first, element.second);
    std::size_t y = std::max(element.first, element.second);

    return x * 779230947 + y * 247091631;
  }
};

struct VertexPairEqualsFunction {
  bool operator()(const std::pair<VolumeVertexIndex, VolumeVertexIndex>& a,
                  const std::pair<VolumeVertexIndex,
                                  VolumeVertexIndex>& b) const {
    VolumeVertexIndex ax = std::min(a.first, a.second);
    VolumeVertexIndex ay = std::max(a.first, a.second);
    VolumeVertexIndex bx = std::min(b.first, b.second);
    VolumeVertexIndex by = std::max(b.first, b.second);

    return (ax == bx) && (ay == by);
  }
};

// Bootstrapping for creating a new vertex in the mesh
// Determinges the boundary type, chooses the type of projection (if any),
// adds the vertex to the mesh data structures, and hashes the new
// vertex in the parents -> child map
template<typename T>
void CreateNewVertex(const VolumeVertexIndex &a,
                     const VolumeVertexIndex &b,
                     std::vector<VolumeVertex<T>> &split_mesh_vertices,
                     std::vector<CylinderVertexType> &split_is_boundary,
                     std::unordered_map<std::pair<VolumeVertexIndex,
                                                  VolumeVertexIndex>,
                                        VolumeVertexIndex,
                                        VertexPairHashFunction,
                                        VertexPairEqualsFunction> &vertex_map,
                     const double radius,
                     const double height) {
  const CylinderVertexType
      p_vertex_type = std::min(split_is_boundary[a], split_is_boundary[b]);

  const Vector3<T> &A = split_mesh_vertices[a].r_MV();
  const Vector3<T> &B = split_mesh_vertices[b].r_MV();

  const Vector3<T> p = ChooseProjection(A, B, p_vertex_type, radius, height);

  const std::pair<VolumeVertexIndex, VolumeVertexIndex>
      p_parents = std::make_pair(a, b);

  vertex_map.insert(std::make_pair(p_parents,
      VolumeVertexIndex(split_mesh_vertices.size())));

  split_mesh_vertices.emplace_back(p);
  split_is_boundary.emplace_back(p_vertex_type);
}

// Refines a tetrahedron defined by vertices A, B, C, and D
// into 8 new tetrahedra. Vector is_boundary stores the
// CylinderVertexType, which determines if and how the generated
// vertices are projected onto the boundary.
// The return is a pair with: 1) a VolumeMesh including the original and
// new vertices, and 2) a vector of booleans that, similar to is_boundary,
// storing the CylinderVertexType of all vertices.
template<typename T>
void RefineCylinderTetrahdron(
    const VolumeElement &tet,
    std::vector<VolumeVertex<T>> &split_mesh_vertices,
    std::vector<VolumeElement> &split_mesh_tetrahedra,
    std::vector<CylinderVertexType> &split_is_boundary,
    std::unordered_map<std::pair<VolumeVertexIndex, VolumeVertexIndex>,
                       VolumeVertexIndex,
                       VertexPairHashFunction,
                       VertexPairEqualsFunction> &vertex_map,
    const double radius,
    const double height) {

  // Index a corresponds to vertex A, index b to vertex B, etc.
  const VolumeVertexIndex a = tet.vertex(0);
  const VolumeVertexIndex b = tet.vertex(1);
  const VolumeVertexIndex c = tet.vertex(2);
  const VolumeVertexIndex d = tet.vertex(3);

  // 6 new vertices are created, each as the midpoint of every pair of
  // vertices in a tet. For example, vertex e is created as the midpoint of
  // vertices a and b. Once one tet generates this vertex, we do not want
  // another tet that shares the common vertices a and b to duplicate the
  // vertex e. Therefore, we store the index of e in split_mesh_vertices
  // in the unordered_map "vertex_map". The key for a vertex in this map
  // is a std::pair of the indices of its parents. The hash for this map is
  // commutative, that is the key (a,b) hashes to the same index as the key
  // (b,a) and returns true when checking for equivalence of the keys.

  const std::pair<VolumeVertexIndex, VolumeVertexIndex>
      &e_parents = std::make_pair(a, b);
  const std::pair<VolumeVertexIndex, VolumeVertexIndex>
      &f_parents = std::make_pair(a, c);
  const std::pair<VolumeVertexIndex, VolumeVertexIndex>
      &g_parents = std::make_pair(a, d);
  const std::pair<VolumeVertexIndex, VolumeVertexIndex>
      &h_parents = std::make_pair(b, c);
  const std::pair<VolumeVertexIndex, VolumeVertexIndex>
      &i_parents = std::make_pair(b, d);
  const std::pair<VolumeVertexIndex, VolumeVertexIndex>
      &j_parents = std::make_pair(c, d);

  auto e_index = vertex_map.find(e_parents);
  auto f_index = vertex_map.find(f_parents);
  auto g_index = vertex_map.find(g_parents);
  auto h_index = vertex_map.find(h_parents);
  auto i_index = vertex_map.find(i_parents);
  auto j_index = vertex_map.find(j_parents);

  // If the vertex does not exist in the map, create a new vertex and add it to
  // the storage vectors as well as the hash map

  if (e_index == vertex_map.end()) {
    CreateNewVertex(a,
                    b,
                    split_mesh_vertices,
                    split_is_boundary,
                    vertex_map,
                    radius,
                    height);
    e_index = vertex_map.find(e_parents);
  }

  if (f_index == vertex_map.end()) {
    CreateNewVertex(a,
                    c,
                    split_mesh_vertices,
                    split_is_boundary,
                    vertex_map,
                    radius,
                    height);
    f_index = vertex_map.find(f_parents);
  }

  if (g_index == vertex_map.end()) {
    CreateNewVertex(a,
                    d,
                    split_mesh_vertices,
                    split_is_boundary,
                    vertex_map,
                    radius,
                    height);
    g_index = vertex_map.find(g_parents);
  }

  if (h_index == vertex_map.end()) {
    CreateNewVertex(b,
                    c,
                    split_mesh_vertices,
                    split_is_boundary,
                    vertex_map,
                    radius,
                    height);
    h_index = vertex_map.find(h_parents);
  }

  if (i_index == vertex_map.end()) {
    CreateNewVertex(b,
                    d,
                    split_mesh_vertices,
                    split_is_boundary,
                    vertex_map,
                    radius,
                    height);
    i_index = vertex_map.find(i_parents);
  }

  if (j_index == vertex_map.end()) {
    CreateNewVertex(c,
                    d,
                    split_mesh_vertices,
                    split_is_boundary,
                    vertex_map,
                    radius,
                    height);
    j_index = vertex_map.find(j_parents);
  }

  // The index of each of the child vertices
  const VolumeVertexIndex e = e_index->second;
  const VolumeVertexIndex f = f_index->second;
  const VolumeVertexIndex g = g_index->second;
  const VolumeVertexIndex h = h_index->second;
  const VolumeVertexIndex i = i_index->second;
  const VolumeVertexIndex j = j_index->second;

  // The four tetrahedra at the corners.
  split_mesh_tetrahedra.emplace_back(a, e, f, g);
  split_mesh_tetrahedra.emplace_back(b, h, e, i);
  split_mesh_tetrahedra.emplace_back(f, h, c, j);
  split_mesh_tetrahedra.emplace_back(j, g, i, d);

  // TODO(joemasterjohn): Split along EJ, FI and GH and choose
  // the partition with best quality factor as described in [Everett, 1997],
  // Currently we arbitrarily choose to partition along the GH edge.
  split_mesh_tetrahedra.emplace_back(g, h, i, e);
  split_mesh_tetrahedra.emplace_back(g, f, h, e);
  split_mesh_tetrahedra.emplace_back(g, i, h, j);
  split_mesh_tetrahedra.emplace_back(g, h, f, j);
}

// Compute the (uniformly weighted) average of the points in neighbors
template<typename T>
Vector3<T> WeightedAverage(const std::vector<Vector3<T>> &neighbors) {
  T number_neighbors(neighbors.size());

  return
      std::accumulate(neighbors.begin(), neighbors.end(), Vector3<T>(0, 0, 0))
          / number_neighbors;
}

// Iterative Laplacian smoothing
// Find all neighbors of each vertex, and store in a set for each vertex
// If a vertex is an internal type vertex, replace it with the average position
// of all of its neighbors.
template<typename T>
std::pair<VolumeMesh<T>,
          std::vector<CylinderVertexType>> LaplacianSmoothingInterior(
    const VolumeMesh<T> &mesh,
    const std::vector<CylinderVertexType> &is_boundary,
    std::size_t number_iterations) {

  std::vector<VolumeElement> smooth_mesh_tetrahedra;
  std::vector<VolumeVertex<T>> smooth_mesh_vertices;
  std::vector<CylinderVertexType> smooth_is_boundary;

  std::copy(mesh.tetrahedra().begin(), mesh.tetrahedra().end(),
            std::back_inserter(smooth_mesh_tetrahedra));
  std::copy(mesh.vertices().begin(),
            mesh.vertices().end(),
            std::back_inserter(smooth_mesh_vertices));
  std::copy(is_boundary.begin(),
            is_boundary.end(),
            std::back_inserter(smooth_is_boundary));

  std::vector<std::unordered_set<VolumeVertexIndex>>
      neighbors(mesh.vertices().size());

  for (const auto &t : smooth_mesh_tetrahedra) {
    for (std::size_t i = 0; i < 4; i++) {
      for (std::size_t j = 1; j < 4; j++) {
        const VolumeVertexIndex a = t.vertex(i);
        const VolumeVertexIndex b = t.vertex((i + j) % 4);

        neighbors[a].emplace(b);
      }
    }
  }

  for (std::size_t n = 0; n < number_iterations; n++) {
    for (std::size_t i = 0; i < smooth_mesh_vertices.size(); i++) {
      std::vector<Vector3<T>> v_neighbors(neighbors[i].size());

      std::transform(neighbors[i].begin(),
                     neighbors[i].end(),
                     v_neighbors.begin(),
                     [&](const VolumeVertexIndex &index) -> Vector3<T> {
                       return smooth_mesh_vertices[index].r_MV();
                     });

      if (smooth_is_boundary[i] == VERTEX_INTERNAL)
        smooth_mesh_vertices[i] = VolumeVertex<T>(WeightedAverage(v_neighbors));
    }
  }

  return std::make_pair(VolumeMesh<T>(std::move(smooth_mesh_tetrahedra),
                                      std::move(smooth_mesh_vertices)),
                        std::move(smooth_is_boundary));
}

// Splits a mesh by calling RefineCylinderTetrahdron() on each
// tetrahedron of `mesh`. `is_boundary` is a vector describing the
// CylinderVertexType of each vertex in `mesh`
template<typename T>
std::pair<VolumeMesh<T>, std::vector<CylinderVertexType>> RefineCylinderMesh(
    const VolumeMesh<T> &mesh,
    const std::vector<CylinderVertexType> &is_boundary,
    const double radius,
    const double height) {

  // Copy the vertex, tet, and boundary information into the vectors for the
  // new subdivided mesh
  std::vector<VolumeElement> split_mesh_tetrahedra;
  std::vector<VolumeVertex<T>> split_mesh_vertices;
  std::vector<CylinderVertexType> split_is_boundary;

  // Copy the indices and boundary information
  std::copy(mesh.vertices().begin(),
            mesh.vertices().end(),
            std::back_inserter(split_mesh_vertices));
  std::copy(is_boundary.begin(),
            is_boundary.end(),
            std::back_inserter(split_is_boundary));

  auto vertex_map =
      std::unordered_map<std::pair<VolumeVertexIndex, VolumeVertexIndex>,
                         VolumeVertexIndex,
                         VertexPairHashFunction,
                         VertexPairEqualsFunction>(mesh.vertices().size());

  for (const auto &t : mesh.tetrahedra()) {
    RefineCylinderTetrahdron<T>(t,
                                split_mesh_vertices,
                                split_mesh_tetrahedra,
                                split_is_boundary,
                                vertex_map,
                                radius,
                                height);
  }

  return std::make_pair(VolumeMesh<T>(std::move(split_mesh_tetrahedra),
                                      std::move(split_mesh_vertices)),
                        split_is_boundary);
}

// Creates the initial mesh for refinement_level = 0.
// The initial mesh is a rectangular prism with
// x,y,z dimensions: sqrt(2)*radius, sqrt(2)*radius, height
// Initial subdivisions are made based on the aspect ratio
// so that initial tetrahedra are somewhat regular in shape.

// TODO(joemasterjohn): Document this much better for
//  the complicated indexing scheme
template<typename T>
std::pair<VolumeMesh<T>,
          std::vector<CylinderVertexType>>
          MakeCylinderMeshLevel0(const double &height, const double &radius) {
  std::vector<VolumeElement> tetrahedra;
  std::vector<VolumeVertex<T>> vertices;

  std::size_t subdivisions =
      static_cast<std::size_t>(std::max(2.0, std::floor(height / radius)));

  const double top_z = (height / 2.0);
  const double bot_z = -(height / 2.0);

  // Initial configuration of a set of vertices for a
  // level 0 cylinder with 2 subdivisions
  //
  //
  //                +Z   -X
  //                 |   /
  //                 |  v2
  //                 | /
  //                 |/
  //  -Y---v3------v4+------v1---+Y
  //                /|
  //               / |
  //             v0  |
  //             /   |
  //           +X    |   -X
  //                 |   /
  //                 |  v7
  //                 | /
  //                 |/
  //  -Y---v8------v9+------v6---+Y
  //                /|
  //               / |
  //             v5  |
  //             /   |
  //           +X    |    -X
  //                 |   /
  //                 |  v12
  //                 | /
  //                 |/
  //  -Y---v13------v14+------v11---+Y
  //                /|
  //               / |
  //             v10  |
  //             /   |
  //           +X    |
  //                -Z

  for (std::size_t i = 0; i <= subdivisions; i++) {
    const double t = (1.0 * i) / subdivisions;
    const double z = (1 - t) * top_z + (t) * bot_z;

    vertices.emplace_back(radius, 0.0, z);
    vertices.emplace_back(0.0, radius, z);
    vertices.emplace_back(-radius, 0.0, z);
    vertices.emplace_back(0.0, -radius, z);
    vertices.emplace_back(0.0, 0.0, z);
  }

  using V = VolumeVertexIndex;

  for (std::size_t j = 0; j < subdivisions; j++) {
    for (std::size_t i = 0; i < 4; i++) {
      const std::size_t a = 5 * j + i;
      const std::size_t b = 5 * j + ((i + 1) % 4);
      const std::size_t c = 5 * j + 4;
      const std::size_t d = 5 * (j + 1) + i;
      const std::size_t e = 5 * (j + 1) + ((i + 1) % 4);
      const std::size_t f = 5 * (j + 1) + 4;

      tetrahedra.emplace_back(V(a), V(c), V(b), V(f));
      tetrahedra.emplace_back(V(a), V(b), V(e), V(f));
      tetrahedra.emplace_back(V(a), V(e), V(d), V(f));
    }
  }

  // Most vertices start on the side
  // Two are cap vertices
  // There are subdivisions - 2 interval vertices
  std::vector<CylinderVertexType>
      is_boundary(5 * (subdivisions + 1), VERTEX_SIDE);
  is_boundary[4] = VERTEX_CAP;
  is_boundary[5 * subdivisions + 4] = VERTEX_CAP;

  for (std::size_t i = 1; i < subdivisions; i++) {
    is_boundary[5 * i + 4] = VERTEX_INTERNAL;
  }

  return std::make_pair(
      VolumeMesh<T>(std::move(tetrahedra), std::move(vertices)), is_boundary);
}

#endif

/// This method implements a variant of the generator described in
/// [Everett, 1997]. It is based on a recursive refinement of an initial
/// (refinement_level = 0) coarse mesh representation of a cylinder with
/// given height and radius. The initial mesh discretizes a rectangular
/// prism, subdividing to make roughly regular tetrahedra
/// At each refinement level each tetrahedron is split into eight new
/// tetrahedra by splitting each edge in half. When splitting an edge formed
/// by vertices on the surface of the sphere, the newly created vertex is
/// projected back onto the surface of the sphere.
///
/// @throws std::exception if refinement_level is negative.
/// @throws std::exception if height is non-positive.
/// @throws std::exception if radius is non-positive.
///
/// [Everett, 1997]  Everett, M.E., 1997. A three-dimensional spherical mesh
/// generator. Geophysical Journal International, 130(1), pp.193-200.
template<typename T>
VolumeMesh<T> MakeCylinderMesh(double height,
                               double radius,
                               int refinement_level,
                               int smoothing_level) {
  DRAKE_THROW_UNLESS(height > 0);
  DRAKE_THROW_UNLESS(radius > 0);
  DRAKE_THROW_UNLESS(refinement_level >= 0);

  std::pair<VolumeMesh<T>, std::vector<CylinderVertexType>>
      pair = MakeCylinderMeshLevel0<T>(height,
                                       radius);
  VolumeMesh<T> &mesh = pair.first;
  std::vector<CylinderVertexType> &is_boundary = pair.second;

  for (int level = 1; level <= refinement_level; ++level) {
    auto split_pair = RefineCylinderMesh<T>(mesh, is_boundary, radius, height);
    mesh = split_pair.first;
    is_boundary = split_pair.second;
    DRAKE_DEMAND(mesh.vertices().size() == is_boundary.size());
  }

  if (smoothing_level > 0) {
    auto split_pair =
        LaplacianSmoothingInterior(mesh, is_boundary, smoothing_level);

    mesh = split_pair.first;
    is_boundary = split_pair.second;
  }

  return std::move(mesh);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
