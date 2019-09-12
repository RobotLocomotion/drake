#pragma once

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

// Helper methods for MakeCylinderMesh().
#ifndef DRAKE_DOXYGEN_CXX

// Determines the boundary type of a vertex of the cylinder mesh.
// Vertices on the circular boundary of the cap are considered
// CylinderVertexType::kSide vertices so that their children inherit the
// CylinderVertexType::kSide type and are projected on the side.
// Also so children of them and a pure side vertex (one that is not on the
// cap) also inherit the CylinderVertexType::kSide type and are projected to
// the side. The order of the enumerators is important because a child of two
// vertices with different types will inherit the first one in the enumerator
// list.
enum class CylinderVertexType { kInternal, kCap, kSide };

// Project the point p to the side of the cylinder in the XY direction
// The point p is projected along the line perpendicular to the center line
// of the cylinder (z-axis).
// @pre `p` is not on the center line of the cylinder
template <typename T>
Vector3<T> ProjectOntoCylinderSide(const Vector3<T>& p, const double radius) {
  Vector3<T> p_xy = Vector3<T>(p[0], p[1], 0.0);

  DRAKE_DEMAND(p[0] != T(0) || p[1] != T(0));

  p_xy.normalize();
  Vector3<T> cylinder_size_xy = radius * p_xy;
  return Vector3<T>(cylinder_size_xy.x(), cylinder_size_xy.y(), p.z());
}

// Project midpoint of two vertices (either cap or internal) so that its
// length from the center axis is the mean of the two vertices' lengths
// from the center axis. The midpoint is projected along the line
// perpendicular to the center axis of the cylinder (z-axis). The projection
// does not change the z coordinate of the midpoint.
template <typename T>
Vector3<T> ProjectMidpointToMiddleCylinder(const Vector3<T>& p,
                                           const Vector3<T>& q) {
  Vector3<T> midpoint = (p + q) / 2.0;

  // The midpoint is on the center axis of the cylinder. No projection.
  if (midpoint.x() == T(0) && midpoint.y() == T(0)) {
    return midpoint;
  }

  Vector3<T> v_xy = Vector3<T>(midpoint[0], midpoint[1], 0.0);
  Vector3<T> p_xy = Vector3<T>(p[0], p[1], 0.0);
  Vector3<T> q_xy = Vector3<T>(q[0], q[1], 0.0);

  auto desired_length = (p_xy.norm() + q_xy.norm()) / 2.0;

  v_xy.normalize();
  Vector3<T> middle_circle_xy = desired_length * v_xy;
  return Vector3<T>(middle_circle_xy.x(), middle_circle_xy.y(), midpoint.z());
}

// Determine the correct projection based on the boundary type of the vertex
template <typename T>
Vector3<T> ChooseProjection(const Vector3<T>& x, const Vector3<T>& y,
                            CylinderVertexType v_type, const double radius) {
  // Project boundary vertices onto the surface of the cylinder.
  const Vector3<T> v = (x + y) / 2.0;

  // Internal vertices can be projected in the same manner as cap vertices
  // for better quality
  switch (v_type) {
    case CylinderVertexType::kSide:
      return ProjectOntoCylinderSide(v, radius);
    case CylinderVertexType::kCap:
    case CylinderVertexType::kInternal:
      return ProjectMidpointToMiddleCylinder(x, y);
    default:
      return v;
  }
}

// Bootstrapping for creating a new vertex in the mesh
// Determines the boundary type, chooses the type of projection (if any),
// adds the vertex to the mesh data structures, and hashes the new
// vertex in the parents -> child map
template <typename T>
void CreateNewVertex(const VolumeVertexIndex& a, const VolumeVertexIndex& b,
                     std::vector<VolumeVertex<T>>* split_mesh_vertices_ptr,
                     std::vector<CylinderVertexType>* split_is_boundary_ptr,
                     std::unordered_map<SortedPair<VolumeVertexIndex>,
                                        VolumeVertexIndex>* vertex_map_ptr,
                     const double radius) {
  DRAKE_DEMAND(split_mesh_vertices_ptr != nullptr);
  DRAKE_DEMAND(split_is_boundary_ptr != nullptr);
  DRAKE_DEMAND(vertex_map_ptr != nullptr);

  std::vector<VolumeVertex<T>>& split_mesh_vertices = *split_mesh_vertices_ptr;
  std::vector<CylinderVertexType>& split_is_boundary = *split_is_boundary_ptr;

  std::unordered_map<SortedPair<VolumeVertexIndex>, VolumeVertexIndex>&
      vertex_map = *vertex_map_ptr;

  const CylinderVertexType p_vertex_type =
      std::min(split_is_boundary[a], split_is_boundary[b]);

  const Vector3<T>& A = split_mesh_vertices[a].r_MV();
  const Vector3<T>& B = split_mesh_vertices[b].r_MV();

  const Vector3<T> p = ChooseProjection(A, B, p_vertex_type, radius);

  const SortedPair<VolumeVertexIndex> p_parents = MakeSortedPair(a, b);

  vertex_map[p_parents] = VolumeVertexIndex(split_mesh_vertices.size());

  split_mesh_vertices.emplace_back(p);
  split_is_boundary.emplace_back(p_vertex_type);
}

// Refines a tetrahedron into 8 tetrahedra.
// @param[in,out] split_mesh_vertices_ptr
//    Original vertices plus new vertices on return.
// @param[out] split_mesh_tetrahedra_ptr
//    New tetrahedra on return.
// @param[in,out] split_is_boundary_ptr
//    Types of original vertices plus types of new vertices on return.
// @param[in,out] vertex_map_ptr
//    Parents to child map. The mapping for new vertices are added on return.
template <typename T>
void RefineCylinderTetrahdron(
    const VolumeElement& tet,
    std::vector<VolumeVertex<T>>* split_mesh_vertices_ptr,
    std::vector<VolumeElement>* split_mesh_tetrahedra_ptr,
    std::vector<CylinderVertexType>* split_is_boundary_ptr,
    std::unordered_map<SortedPair<VolumeVertexIndex>, VolumeVertexIndex>*
        vertex_map_ptr,
    const double radius) {
  DRAKE_DEMAND(split_mesh_vertices_ptr != nullptr);
  DRAKE_DEMAND(split_mesh_tetrahedra_ptr != nullptr);
  DRAKE_DEMAND(split_is_boundary_ptr != nullptr);
  DRAKE_DEMAND(vertex_map_ptr != nullptr);

  std::vector<VolumeVertex<T>>& split_mesh_vertices = *split_mesh_vertices_ptr;
  std::vector<CylinderVertexType>& split_is_boundary = *split_is_boundary_ptr;
  std::vector<VolumeElement>& split_mesh_tetrahedra =
      *split_mesh_tetrahedra_ptr;

  std::unordered_map<SortedPair<VolumeVertexIndex>, VolumeVertexIndex>&
      vertex_map = *vertex_map_ptr;

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

  const SortedPair<VolumeVertexIndex> e_parents = MakeSortedPair(a, b);
  const SortedPair<VolumeVertexIndex> f_parents = MakeSortedPair(a, c);
  const SortedPair<VolumeVertexIndex> g_parents = MakeSortedPair(a, d);
  const SortedPair<VolumeVertexIndex> h_parents = MakeSortedPair(b, c);
  const SortedPair<VolumeVertexIndex> i_parents = MakeSortedPair(b, d);
  const SortedPair<VolumeVertexIndex> j_parents = MakeSortedPair(c, d);

  auto e_index = vertex_map.find(e_parents);
  auto f_index = vertex_map.find(f_parents);
  auto g_index = vertex_map.find(g_parents);
  auto h_index = vertex_map.find(h_parents);
  auto i_index = vertex_map.find(i_parents);
  auto j_index = vertex_map.find(j_parents);

  // If the vertex does not exist in the map, create a new vertex and add it to
  // the storage vectors as well as the hash map

  if (e_index == vertex_map.end()) {
    CreateNewVertex(a, b, &split_mesh_vertices, &split_is_boundary, &vertex_map,
                    radius);
    e_index = vertex_map.find(e_parents);
  }

  if (f_index == vertex_map.end()) {
    CreateNewVertex(a, c, &split_mesh_vertices, &split_is_boundary, &vertex_map,
                    radius);
    f_index = vertex_map.find(f_parents);
  }

  if (g_index == vertex_map.end()) {
    CreateNewVertex(a, d, &split_mesh_vertices, &split_is_boundary, &vertex_map,
                    radius);
    g_index = vertex_map.find(g_parents);
  }

  if (h_index == vertex_map.end()) {
    CreateNewVertex(b, c, &split_mesh_vertices, &split_is_boundary, &vertex_map,
                    radius);
    h_index = vertex_map.find(h_parents);
  }

  if (i_index == vertex_map.end()) {
    CreateNewVertex(b, d, &split_mesh_vertices, &split_is_boundary, &vertex_map,
                    radius);
    i_index = vertex_map.find(i_parents);
  }

  if (j_index == vertex_map.end()) {
    CreateNewVertex(c, d, &split_mesh_vertices, &split_is_boundary, &vertex_map,
                    radius);
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

// Splits a mesh by calling RefineCylinderTetrahdron() on each
// tetrahedron of `mesh`. `is_boundary` is a vector describing the
// CylinderVertexType of each vertex in `mesh`
template <typename T>
std::pair<VolumeMesh<T>, std::vector<CylinderVertexType>> RefineCylinderMesh(
    const VolumeMesh<T>& mesh,
    const std::vector<CylinderVertexType>& is_boundary, const double radius) {
  // Copy the vertex, and boundary information into the vectors for the
  // new subdivided mesh
  std::vector<VolumeVertex<T>> split_mesh_vertices = mesh.vertices();
  std::vector<CylinderVertexType> split_is_boundary = is_boundary;

  // Original tets are all subdivied, so split_mesh_tetrahedra will only
  // contain new tets.
  std::vector<VolumeElement> split_mesh_tetrahedra;

  // A map from two parent vertices in the original mesh to the new vertex in
  // the refined mesh.
  auto vertex_map =
      std::unordered_map<SortedPair<VolumeVertexIndex>, VolumeVertexIndex>(
          6 * mesh.tetrahedra().size());

  for (const auto& t : mesh.tetrahedra()) {
    RefineCylinderTetrahdron<T>(t, &split_mesh_vertices, &split_mesh_tetrahedra,
                                &split_is_boundary, &vertex_map, radius);
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
template <typename T>
std::pair<VolumeMesh<T>, std::vector<CylinderVertexType>>
MakeCylinderMeshLevel0(const double& height, const double& radius) {
  std::vector<VolumeElement> tetrahedra;
  std::vector<VolumeVertex<T>> vertices;

  int subdivisions =
      static_cast<int>(std::max(2.0, std::floor(height / radius)));

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

  // Groups of 5 vertices are on a slice of the rectangular prism
  // with a plane perpendicular to the Z axisx at a given height "z".
  //
  // "subdivisions" is how many slices perpendicular to the Z axis we
  // make.
  //
  // Every 5th vertex is exactly at (0, 0, z) for a given  height "z" where:
  // bot_z <= z <= top_z.
  for (int i = 0; i <= subdivisions; i++) {
    const double t = (1.0 * i) / subdivisions;
    const double z = (1 - t) * top_z + (t)*bot_z;

    vertices.emplace_back(radius, 0.0, z);
    vertices.emplace_back(0.0, radius, z);
    vertices.emplace_back(-radius, 0.0, z);
    vertices.emplace_back(0.0, -radius, z);
    vertices.emplace_back(0.0, 0.0, z);
  }

  using V = VolumeVertexIndex;

  // Each j-th iteration adds 12 tetrahedra of a rectangular prism of one
  // subdivision.
  // Each i-th iteration add 3 tetrahedra of a triangular prism, four of which
  // makes a rectangular prism.
  for (int j = 0; j < subdivisions; j++) {
    for (int i = 0; i < 4; i++) {
      const int a = 5 * j + i;
      const int b = 5 * j + ((i + 1) % 4);
      const int c = 5 * j + 4;
      const int d = 5 * (j + 1) + i;
      const int e = 5 * (j + 1) + ((i + 1) % 4);
      const int f = 5 * (j + 1) + 4;

      tetrahedra.emplace_back(V(a), V(c), V(b), V(f));
      tetrahedra.emplace_back(V(a), V(b), V(e), V(f));
      tetrahedra.emplace_back(V(a), V(e), V(d), V(f));
    }
  }

  // Most vertices start on the side
  // Two are cap vertices
  // There are subdivisions - 1 internal vertices
  std::vector<CylinderVertexType> is_boundary(5 * (subdivisions + 1),
                                              CylinderVertexType::kSide);
  is_boundary[4] = CylinderVertexType::kCap;
  is_boundary[5 * subdivisions + 4] = CylinderVertexType::kCap;

  for (int i = 1; i < subdivisions; i++) {
    is_boundary[5 * i + 4] = CylinderVertexType::kInternal;
  }

  return std::make_pair(
      VolumeMesh<T>(std::move(tetrahedra), std::move(vertices)), is_boundary);
}

#endif
/// Generates a tetrahedral volume mesh of a cylinder whose bounding box is
/// [-radius, radius]x[-radius,radius]x[-height/2, height/2], i.e., the center
/// line of the cylinder is the z-axis, and the center of the cylinder is at
/// the origin.
///
/// This method implements a variant of the generator described in
/// [Everett, 1997]. The algorithm has diverged a bit from the one in the paper,
/// but the main ideas used from the paper are:
///
///   - the pattern of subdividing edges to create vertices
///   - the projection of surface vertices
///   - the consideration of the combinatorics of how to associate new
///     subdivision tetrahedra to the newly created vertices.
///
/// It is based on a recursive refinement of an initial
/// (refinement_level = 0) coarse mesh representation of a cylinder with
/// given height and radius. The initial mesh discretizes a rectangular
/// prism, subdividing to make roughly regular tetrahedra.
/// At each refinement level, each tetrahedron is split into eight new
/// tetrahedra by splitting each edge in half.
///
/// When splitting an edge, the midpoint of the edge is projected along the line
/// orthogonal to the z-axis. For an edge on the side surface of the cylinder,
/// the newly created vertex is placed on the side surface. For other kinds
/// of edges, the projection places the newly created vertex at the distance
/// from the z-axis equal to the average of the two distances from the z-axis
/// of the two original vertices of the split edge.  If the two original
/// vertices have the same distance from the z-axis, the newly created vertex
/// will have the same distance from the z-axis as the original vertices.  As
/// a result, the mesh vertices are placed on the 2ⁿ concentric cylindrical
/// surfaces inside the cylinder, where n is the `refinement_level`.
///
/// @param[in] cylinder
///    Specification of the parameterized cylinder the output mesh should
///    approximate.
/// @param[in] refinement_level
///    The number of subdivision steps to take. If the original mesh contains
///    N tetrahedra, the resulting mesh with refinement_level = L will contain
///    N·8ᴸ tetrahedra.
///
/// @throws std::exception if refinement_level is negative.
///
/// [Everett, 1997]  Everett, M.E., 1997. A three-dimensional spherical mesh
/// generator. Geophysical Journal International, 130(1), pp.193-200.
template <typename T>
VolumeMesh<T> MakeCylinderMesh(const Cylinder& cylinder, int refinement_level) {
  const double height = cylinder.get_length();
  const double radius = cylinder.get_radius();

  DRAKE_THROW_UNLESS(refinement_level >= 0);

  std::pair<VolumeMesh<T>, std::vector<CylinderVertexType>> pair =
      MakeCylinderMeshLevel0<T>(height, radius);
  VolumeMesh<T>& mesh = pair.first;
  std::vector<CylinderVertexType>& is_boundary = pair.second;

  for (int level = 1; level <= refinement_level; ++level) {
    auto split_pair = RefineCylinderMesh<T>(mesh, is_boundary, radius);
    mesh = split_pair.first;
    is_boundary = split_pair.second;
    DRAKE_DEMAND(mesh.vertices().size() == is_boundary.size());
  }

  return std::move(mesh);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
