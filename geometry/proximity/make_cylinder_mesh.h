#pragma once

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

// Helper methods for MakeCylinderVolumeMesh().
#ifndef DRAKE_DOXYGEN_CXX

// TODO(DamrongGuoy): Consider removing the classification of vertex types.
//  We could probably remove ProjectOntoCylinderSide() and use only
//  ProjectMidpointToMiddleCylinder() for the midpoints of all edges on the
//  side, on the cap, or in the interior volume.

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

// TODO(DamrongGuoy): Consider removing ProjectOntoCylinderSide().

// Projects the point p to the side of the cylinder in the XY direction.
// The point p is projected along the line perpendicular to the center line
// of the cylinder (z-axis).
// @pre This function is supposed to be used only on the outer-most shell, so
//      we demand that `p` is at least half the `radius` from the center line
//      of the cylinder.
template <typename T>
Vector3<T> ProjectOntoCylinderSide(const Vector3<T>& p, const double radius) {
  Vector2<T> p_xy = Vector2<T>(p[0], p[1]);

  T norm = p_xy.norm();
  DRAKE_DEMAND(norm >= T(radius / 2.0));

  p_xy /= norm;
  Vector2<T> cylinder_size_xy = radius * p_xy;
  return Vector3<T>(cylinder_size_xy.x(), cylinder_size_xy.y(), p.z());
}

// Projects the midpoint between two vertices `p` and `q` to the cylinder
// halfway between the cylinder passing through `p` and the cylinder passing
// through `q`. If `p` and `q` are on the same cylinder, the projection is on
// the common cylinder. The vertices could be a cap vertex or an internal
// vertex.  The midpoint is projected along the line perpendicular to the
// center axis of the cylinder (z-axis). The projection does not change the
// z-coordinate of the midpoint.
template <typename T>
Vector3<T> ProjectMidpointToMiddleCylinder(const Vector3<T>& p,
                                           const Vector3<T>& q) {
  Vector3<T> midpoint = (p + q) / 2.0;

  Vector2<T> v_xy = Vector2<T>(midpoint[0], midpoint[1]);
  // The midpoint is on or near the center axis of the cylinder. No projection.
  T kEps = std::numeric_limits<double>::epsilon();
  if (v_xy.squaredNorm() <= kEps * kEps) {
    return midpoint;
  }

  Vector2<T> p_xy = Vector2<T>(p[0], p[1]);
  Vector2<T> q_xy = Vector2<T>(q[0], q[1]);

  auto desired_length = (p_xy.norm() + q_xy.norm()) / 2.0;

  v_xy.normalize();
  Vector2<T> middle_circle_xy = desired_length * v_xy;
  return Vector3<T>(middle_circle_xy.x(), middle_circle_xy.y(), midpoint.z());
}

// Projects the midpoint between two vertices based on the boundary type of the
// vertex.
template <typename T>
Vector3<T> ProjectMidPoint(const Vector3<T>& x, const Vector3<T>& y,
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

// TODO(DamrongGuoy): Remove `split_vertex_type_ptr`. Replace two functions
//  ProjectMidpointToMiddleCylinder() and ProjectMidPoint() with one function
//  that is called from CreateNewVertex() without relying on vertex type.

// Bootstrapping for creating a new vertex in the mesh
// Determines the boundary type, chooses the type of projection (if any),
// adds the vertex to the mesh data structures, and hashes the new
// vertex in the parents -> child map
template <typename T>
VolumeVertexIndex CreateNewVertex(
    VolumeVertexIndex a, VolumeVertexIndex b,
    std::vector<VolumeVertex<T>>* split_mesh_vertices_ptr,
    std::vector<CylinderVertexType>* split_vertex_type_ptr,
    std::unordered_map<SortedPair<VolumeVertexIndex>, VolumeVertexIndex>*
        vertex_map_ptr,
    const double radius) {
  DRAKE_DEMAND(split_mesh_vertices_ptr != nullptr);
  DRAKE_DEMAND(split_vertex_type_ptr != nullptr);
  DRAKE_DEMAND(vertex_map_ptr != nullptr);

  std::vector<VolumeVertex<T>>& split_mesh_vertices = *split_mesh_vertices_ptr;
  std::vector<CylinderVertexType>& split_vertex_type = *split_vertex_type_ptr;
  std::unordered_map<SortedPair<VolumeVertexIndex>, VolumeVertexIndex>&
      vertex_map = *vertex_map_ptr;

  const CylinderVertexType p_vertex_type =
      std::min(split_vertex_type[a], split_vertex_type[b]);

  const Vector3<T>& A = split_mesh_vertices[a].r_MV();
  const Vector3<T>& B = split_mesh_vertices[b].r_MV();

  const Vector3<T> p = ProjectMidPoint(A, B, p_vertex_type, radius);

  const SortedPair<VolumeVertexIndex> p_parents = MakeSortedPair(a, b);

  const VolumeVertexIndex new_vertex_index(split_mesh_vertices.size());
  vertex_map[p_parents] = new_vertex_index;

  split_mesh_vertices.emplace_back(p);
  split_vertex_type.push_back(p_vertex_type);

  return new_vertex_index;
}

// Refines a tetrahedron into 8 tetrahedra.
// @param[in] tet
//    The tetrahedron to refine.
// @param[in,out] split_mesh_vertices_ptr
//    Original vertices plus new vertices on return.
// @param[out] split_mesh_tetrahedra_ptr
//    Original tetrahedra plus new tetrahedra on return.
// @param[in,out] split_vertex_type_ptr
//    Types of original vertices plus types of new vertices on return.
// @param[in,out] vertex_map_ptr
//    Parents to child map. The mapping for new vertices are added on return.
// @param[in] radius
//    Radius of the cylinder.
template <typename T>
void RefineCylinderTetrahdron(
    const VolumeElement& tet,
    std::vector<VolumeVertex<T>>* split_mesh_vertices_ptr,
    std::vector<VolumeElement>* split_mesh_tetrahedra_ptr,
    std::vector<CylinderVertexType>* split_vertex_type_ptr,
    std::unordered_map<SortedPair<VolumeVertexIndex>, VolumeVertexIndex>*
        vertex_map_ptr,
    const double radius) {
  DRAKE_DEMAND(split_mesh_vertices_ptr != nullptr);
  DRAKE_DEMAND(split_mesh_tetrahedra_ptr != nullptr);
  DRAKE_DEMAND(split_vertex_type_ptr != nullptr);
  DRAKE_DEMAND(vertex_map_ptr != nullptr);

  std::vector<VolumeVertex<T>>& split_mesh_vertices = *split_mesh_vertices_ptr;
  std::vector<CylinderVertexType>& split_vertex_type = *split_vertex_type_ptr;
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

  auto get_child_vertex = [&vertex_map, &split_mesh_vertices,
                           &split_vertex_type,
                           &radius](VolumeVertexIndex p, VolumeVertexIndex q) {
    SortedPair<VolumeVertexIndex> parents{p, q};
    auto iter = vertex_map.find(parents);
    if (iter != vertex_map.end()) return iter->second;
    return CreateNewVertex(parents.first(), parents.second(),
                           &split_mesh_vertices, &split_vertex_type,
                           &vertex_map, radius);
  };

  // The index of each of the child vertices
  const VolumeVertexIndex e = get_child_vertex(a, b);
  const VolumeVertexIndex f = get_child_vertex(a, c);
  const VolumeVertexIndex g = get_child_vertex(a, d);
  const VolumeVertexIndex h = get_child_vertex(b, c);
  const VolumeVertexIndex i = get_child_vertex(b, d);
  const VolumeVertexIndex j = get_child_vertex(c, d);

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

/// Splits a mesh by calling RefineCylinderTetrahdron() on each
/// tetrahedron of `mesh`. `vertex_type` is a vector describing the
/// CylinderVertexType of each vertex in `mesh`
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
template <typename T>
std::pair<VolumeMesh<T>, std::vector<CylinderVertexType>> RefineCylinderMesh(
    const VolumeMesh<T>& mesh,
    const std::vector<CylinderVertexType>& vertex_type, const double radius) {
  // Copy the vertex, and boundary information into the vectors for the
  // new subdivided mesh
  std::vector<VolumeVertex<T>> split_mesh_vertices = mesh.vertices();
  std::vector<CylinderVertexType> split_vertex_type = vertex_type;

  // Original tets are all subdivied, so split_mesh_tetrahedra will only
  // contain new tets.
  std::vector<VolumeElement> split_mesh_tetrahedra;

  // A map from two parent vertices in the original mesh to the new vertex in
  // the refined mesh.
  std::unordered_map<SortedPair<VolumeVertexIndex>, VolumeVertexIndex>
      vertex_map(6 * mesh.num_elements());

  for (const auto& t : mesh.tetrahedra()) {
    RefineCylinderTetrahdron<T>(t, &split_mesh_vertices, &split_mesh_tetrahedra,
                                &split_vertex_type, &vertex_map, radius);
  }

  return std::make_pair(VolumeMesh<T>(std::move(split_mesh_tetrahedra),
                                      std::move(split_mesh_vertices)),
                        split_vertex_type);
}

// Creates the initial mesh for refinement_level = 0.
// The initial mesh is a rectangular prism with
// x,y,z dimensions: sqrt(2)*radius, sqrt(2)*radius, height,
// so that the diagonal on the xy plane has length 2 * radius.
template <typename T>
std::pair<VolumeMesh<T>, std::vector<CylinderVertexType>>
MakeCylinderMeshLevel0(const double& height, const double& radius) {
  std::vector<VolumeElement> tetrahedra;
  std::vector<VolumeVertex<T>> vertices;

  // Initial subdivisions along the length of the cylinder are made based on
  // the aspect ratio so that, for a long cylinder, initial tetrahedra are
  // somewhat regular in shape. However, for a short cylinder (like a disk),
  // there are 2 subdivisions, so we have at least one interior vertex and
  // avoid forcing the extent field to be zero everywhere.
  int subdivisions =
      static_cast<int>(std::max(2.0, std::floor(height / radius)));

  const double top_z = height / 2.0;
  const double bot_z = -top_z;

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
  //  -Y---v13----v14+------v11---+Y
  //                /|
  //               / |
  //             v10 |
  //             /   |
  //           +X    |
  //                -Z

  // Groups of 5 vertices are on a slice of the rectangular prism
  // with a plane perpendicular to the Z axis at a given height "z".
  //
  // "subdivisions" is how many slices perpendicular to the Z axis we
  // make.
  //
  // Every 5th vertex is exactly at (0, 0, z) for a given height "z" where:
  // bot_z <= z <= top_z.
  auto add_slice_vertices = [&vertices, &radius](double z) {
    vertices.emplace_back(radius, 0.0, z);
    vertices.emplace_back(0.0, radius, z);
    vertices.emplace_back(-radius, 0.0, z);
    vertices.emplace_back(0.0, -radius, z);
    vertices.emplace_back(0.0, 0.0, z);
  };
  const double slab_height = height / subdivisions;
  for (int i = 0; i < subdivisions; i++) {
    add_slice_vertices(top_z - i * slab_height);
  }
  add_slice_vertices(bot_z);

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
  std::vector<CylinderVertexType> vertex_type(5 * (subdivisions + 1),
                                              CylinderVertexType::kSide);
  vertex_type[4] = CylinderVertexType::kCap;
  vertex_type[5 * subdivisions + 4] = CylinderVertexType::kCap;

  for (int i = 1; i < subdivisions; i++) {
    vertex_type[5 * i + 4] = CylinderVertexType::kInternal;
  }

  return std::make_pair(
      VolumeMesh<T>(std::move(tetrahedra), std::move(vertices)), vertex_type);
}

#endif

/// Generates a tetrahedral volume mesh of a cylinder whose bounding box is
/// [-radius, radius]x[-radius,radius]x[-length/2, length/2], i.e., the center
/// line of the cylinder is the z-axis, and the center of the cylinder is at
/// the origin.
///
/// The level of tessellation is guided by the `resolution_hint` parameter.
/// Smaller values create higher-resolution meshes with smaller tetrahedra.
/// The resolution hint is interpreted as an edge length, and the cylinder is
/// subdivided to guarantee that edge lengths along the boundary circle of
/// the top and bottom caps will be less than or equal to that edge length.
///
/// The resolution of the final mesh will change discontinuously. Small
/// changes to `resolution_hint` will likely produce the same mesh.
/// However, cutting `resolution_hint` in half will likely increase the
/// number of tetrahedra.
///
/// Ultimately, successively smaller values of `resolution_hint` will no
/// longer change the output mesh. This algorithm will not produce more than
/// 100 million tetrahedra.
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
/// As edges get split, cylindrical shells are created. Each shell layer
/// conforms to a cylindrical offset surface of the outer cylindrical surface.
///
/// @param[in] cylinder
///    Specification of the parameterized cylinder the output mesh should
///    approximate.
/// @param[in] resolution_hint
///    The positive characteristic edge length for the mesh. The coarsest
///    possible mesh (a rectangular prism) is guaranteed for any value of
///    `resolution_hint` greater than √2 times the radius of the cylinder.
/// @tparam T
///    The Eigen-compatible scalar for representing the mesh vertex positions.
/// @pre resolution_hint is positive.
///
/// [Everett, 1997]  Everett, M.E., 1997. A three-dimensional spherical mesh
/// generator. Geophysical Journal International, 130(1), pp.193-200.
template <typename T>
VolumeMesh<T> MakeCylinderVolumeMesh(const Cylinder& cylinder,
                                     double resolution_hint) {
  const double length = cylinder.length();
  const double radius = cylinder.radius();
  std::pair<VolumeMesh<T>, std::vector<CylinderVertexType>> pair =
      MakeCylinderMeshLevel0<T>(length, radius);
  VolumeMesh<T>& mesh = pair.first;
  std::vector<CylinderVertexType>& vertex_type = pair.second;

  /*
    Calculate the refinement level `L` to satisfy the resolution hint, which
    bounds the length `e` of mesh edges on the boundary circle of the top
    and bottom caps.
        The volume mesh is formed by successively refining a rectangular
    prism.  At the boundary circle of the top and bottom caps, we go from 4
    edges, to 8 edges, to 16 edges, etc., doubling at each level of
    refinement. These edges are simply chords across fixed-length arcs of the
    circle. Based on that we can easily compute the length `e` of the chord
    and bound it by resolution_hint.
        We can calculate `e` from the radius r of the cylinder and the central
    angle θ supported by the chord in this picture:

                    x x x x x
                 x      | \    x
               x        |   \    x
             x          |     \    x
           x            |       \    x
          x    radius r |       e \   x
         x              |           \  x
        x               |             \ x
        x               | θ             \
        x               +---------------x
        x                   radius r    x
        x                               x
         x                             x
          x                           x
           x                         x
             x                     x
               x                 x
                 x             x
                    x x x x x

    The chord length e = 2⋅r⋅sin(θ/2). Solving for θ we get: θ = 2⋅sin⁻¹(e/2⋅r).

    We can relate θ with the refinement level L.

       θ = 2π / 4⋅2ᴸ
         = π / 2⋅2ᴸ
         = π / 2ᴸ⁺¹

     Substituting for θ, we get:

       π / 2ᴸ⁺¹ = 2⋅sin⁻¹(e/2⋅r)
       2ᴸ⁺¹ = π / 2⋅sin⁻¹(e/2⋅r)
       L + 1 = log₂(π / 2⋅sin⁻¹(e/2⋅r))
       L = log₂(π / 2⋅sin⁻¹(e/2⋅r)) - 1
       L = ⌈log₂(π / sin⁻¹(e/2⋅r))⌉ - 2
   */
  DRAKE_DEMAND(resolution_hint > 0.0);
  // Make sure the arcsin doesn't blow up.
  const double e = std::min(resolution_hint, 2.0 * radius);
  const int L = std::max(
      0, static_cast<int>(
             std::ceil(std::log2(M_PI / std::asin(e / (2.0 * radius)))) - 2));

  // TODO(DamrongGuoy): Reconsider the limit of 100 million tetrahedra.
  //  Should it be smaller like 1 million? It will depend on how the system
  //  perform in practice.

  // Limit refinement_level to 100 million tetrahedra. Each refinement level
  // increases the number of tetrahedra by a factor of 8. Let N₀ be the
  // number of initial tetrahedra. The number of tetrahedra N with refinement
  // level L would be:
  //   N = N₀ * 8ᴸ
  //   L = log₈(N/N₀)
  // We can limit N to 100 million by:
  //   L ≤ log₈(1e8/N₀) = log₂(1e8/N₀)/3
  const int refinement_level = std::min(
      L,
      static_cast<int>(std::floor(std::log2(1.e8 / mesh.num_elements()) / 3.)));

  // If the original mesh contains N tetrahedra, the resulting mesh with
  // refinement_level = L will contain N·8ᴸ tetrahedra.
  for (int level = 1; level <= refinement_level; ++level) {
    auto split_pair = RefineCylinderMesh<T>(mesh, vertex_type, radius);
    mesh = split_pair.first;
    vertex_type = split_pair.second;
    DRAKE_DEMAND(mesh.vertices().size() == vertex_type.size());
  }

  return mesh;
}

/// Creates a surface mesh for the given `cylinder`; the level of
/// tessellation is guided by the `resolution_hint` parameter in the same way
/// as MakeCylinderVolumeMesh.
///
/// @param[in] cylinder
///    Specification of the parameterized cylinder the output surface mesh
///    should approximate.
/// @param[in] resolution_hint
///    The positive characteristic edge length for the mesh. The coarsest
///    possible mesh (a rectangular prism) is guaranteed for any value of
///    `resolution_hint` greater than √2 times the radius of the cylinder.
///    For any cylinder, there is a `resolution_hint` value that serves as the
///    smallest value that will produce more triangles -- values smaller than
///    that will have no effect.
/// @returns The triangulated surface mesh for the given cylinder.
/// @pre resolution_hint is positive.
/// @tparam T
///    The Eigen-compatible scalar for representing the mesh vertex positions.
template <typename T>
SurfaceMesh<T> MakeCylinderSurfaceMesh(const Cylinder& cylinder,
                                       double resolution_hint) {
  DRAKE_DEMAND(resolution_hint > 0.0);
  return ConvertVolumeToSurfaceMesh<T>(
      MakeCylinderVolumeMesh<T>(cylinder, resolution_hint));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
