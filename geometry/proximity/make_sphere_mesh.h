#pragma once

#include <algorithm>
#include <array>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/tessellation_strategy.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/* Makes the initial mesh for refinement_level = 0.
 It creates an octahedron by placing its six vertices on the surface of the
 unit sphere and an additional vertex at the origin. The volume is then
 tessellated into eight tetrahedra.
 The additional vector of booleans indicates `true` if the corresponding
 vertex lies on the surface of the sphere.
 Each tet is constructed such that the common vertex (the origin) is the fourth
 vertex.  */
template <typename T>
std::pair<VolumeMesh<T>, std::vector<bool>> MakeSphereMeshLevel0() {
  std::vector<VolumeElement> tetrahedra;
  std::vector<Vector3<T>> vertices;

  // Level "0" consists of the octahedron with vertices on the surface of the
  // a sphere of unit radius, according to the diagram below:
  //                +Z   -X
  //                 |   /
  //                 v5 v3
  //                 | /
  //                 |/
  //  -Y---v4------v0+------v2---+Y
  //                /|
  //               / |
  //             v1  v6
  //             /   |
  //           +X    |
  //                -Z
  vertices.emplace_back(0.0, 0.0, 0.0);   // v0
  vertices.emplace_back(1.0, 0.0, 0.0);   // v1
  vertices.emplace_back(0.0, 1.0, 0.0);   // v2
  vertices.emplace_back(-1.0, 0.0, 0.0);  // v3
  vertices.emplace_back(0.0, -1.0, 0.0);  // v4
  vertices.emplace_back(0.0, 0.0, 1.0);   // v5
  vertices.emplace_back(0.0, 0.0, -1.0);  // v6

  // Create tetrahedra. The convention is that the first three vertices define
  // the "base" of the tetrahedron with its right-handed normal vector
  // pointing towards the inside. The fourth vertex is on the "positive" side
  // of the plane defined by this normal.

  // Top tetrahedra.
  tetrahedra.emplace_back(1, 5, 2, 0);
  tetrahedra.emplace_back(2, 5, 3, 0);
  tetrahedra.emplace_back(3, 5, 4, 0);
  tetrahedra.emplace_back(4, 5, 1, 0);

  // Bottom tetrahedra.
  tetrahedra.emplace_back(2, 6, 1, 0);
  tetrahedra.emplace_back(3, 6, 2, 0);
  tetrahedra.emplace_back(4, 6, 3, 0);
  tetrahedra.emplace_back(1, 6, 4, 0);

  // Indicate what vertices are on the surface of the sphere.
  // All vertices are boundaries but the first one at (0, 0, 0).
  std::vector<bool> is_boundary(7, true);
  is_boundary[0] = false;

  return std::make_pair(
      VolumeMesh<T>(std::move(tetrahedra), std::move(vertices)), is_boundary);
}

/* Given the six vertices that form the interior octahedron, decomposes the
 octahedron into the "best" set of four tetrahedra. The vertices are encoded
 by six vertex indices into the collection of vertex positions. The resulting
 tetrahedra are appended to `tets`.

 The input vertex indices must be ordered in a very particular order. Each
 represents the vertex that splits the edge ij. So, using the edge name as
 a synonym for the vertex that splits it, the order must be: ab, ac, ad, bc, bd,
 and cd (otherwise known as e, f, g, h, i, and j in `SplitTetrahedron`).

 @param vertex_indices      The ordered indices of the six vertices that form
                            the octahedron.
 @param p_MVs               The position vector of each vertex V in the mesh
                            frame M -- indexed by `vertex_indices`.
 @param[inout] tets         The collection of tetrahedra; the best four
                            tetrahedra will be appended to this data.
 @tparam T  The Eigen-compatible scalar for representing vertex positions.
 */
template <typename T>
void SplitOctohedron(const std::array<int, 6>& vertex_indices,
                     const std::vector<Vector3<T>>& p_MVs,
                     std::vector<VolumeElement>* tets) {
  // The interior octahedron can be split along three different axes: EJ, FI,
  // or GH. Ideally, we should look at the decomposition that arises from each
  // split axis and pick the axis that provides the best quality (as seen in
  // [Everett, 1997]). Several important notes:
  //
  //   1. [Everett, 1997] has *two* mistakes in the quality metric. It gives
  //      quality as: Q = 12(3V)^(3/2)⋅∑lᵢᵀ⋅lᵢ. The paper it cites presents it
  //      as: η = 12(3v)^(2/3)/∑lᵢᵀ⋅lᵢ.
  //      - note the incorrect exponent *and* the multiplication in place of
  //        the division.
  //      - In Everett's mistaken version, Q is *not* unitless. In the original,
  //        it is.
  //   2. Everett advocates maximizing the sum of Q across all four candidate
  //      tetrahedra for the split axis.
  //   3. For this purpose, the constants 12 and 3^(2/3) are pointless and can
  //      be omitted to determine *relative* quality: Q = v^(2/3)/∑lᵢᵀ⋅lᵢ.
  //   4. v^2 and ∑lᵢᵀ⋅lᵢ are strictly positive. So, we can get rid of the
  //      cube-root by defining Q' = v²/(∑lᵢᵀ⋅lᵢ)³.

  // Pre-computes all squared edge lengths.
  std::unordered_map<SortedPair<int>, double> square_edge_len;
  for (int i = 0; i < 6; ++i) {
    for (int j = i + 1; j < 6; ++j) {
      const int a = vertex_indices[i];
      const int b = vertex_indices[j];
      square_edge_len[SortedPair<int>(a, b)] = ExtractDoubleOrThrow(
          (p_MVs[a] - p_MVs[b]).squaredNorm());
    }
  }

  // Computes the volume of a single tetrahedron.
  auto volume = [&p_MVs](int a, int b, int c, int d) {
    const Vector3<T> p_AB = p_MVs[b] - p_MVs[a];
    const Vector3<T> p_AC = p_MVs[c] - p_MVs[a];
    const Vector3<T> p_AD = p_MVs[d] - p_MVs[a];
    return ExtractDoubleOrThrow(p_AB.cross(p_AC).dot(p_AD) / 6.0);
  };

  // Computes the quality (Q') of a single tetrahedron given function-local
  // vertex indices (i.e., in the range [0, 5]).
  auto tet_quality = [&square_edge_len, &volume,
                      &vertex_indices](const std::array<int, 4>& local_i) {
    const int a = vertex_indices[local_i[0]];
    const int b = vertex_indices[local_i[1]];
    const int c = vertex_indices[local_i[2]];
    const int d = vertex_indices[local_i[3]];
    const double v = volume(a, b, c, d);
    double denom = 0.0;
    denom += square_edge_len.at(SortedPair<int>(a, b));
    denom += square_edge_len.at(SortedPair<int>(a, c));
    denom += square_edge_len.at(SortedPair<int>(a, d));
    denom += square_edge_len.at(SortedPair<int>(b, c));
    denom += square_edge_len.at(SortedPair<int>(b, d));
    denom += square_edge_len.at(SortedPair<int>(c, d));
    return v * v / (denom * denom * denom);
  };

  // Computes the quality of a split given the function-local tetrahedron
  // indices for all four tetrahedra.
  auto split_quality =
      [&tet_quality](const std::array<std::array<int, 4>, 4>& tets_in) {
        double cost = 0;
        for (const auto& tet : tets_in) cost += tet_quality(tet);
        return cost;
      };

  // These data structures enumerate the tets based on a splitting edge. They
  // are indices into `vertex_indices`. Assuming `vertex_indices` is properly
  // ordered, these local indices are invariant.
  const int e(0), f(1), g(2), h(3), i(4), j(5);
  // clang-format off
  const std::array<std::array<int, 4>, 4> kGhTets{{{g, h, i, e},
                                                   {g, f, h, e},
                                                   {g, i, h, j},
                                                   {g, h, f, j}}};
  const std::array<std::array<int, 4>, 4> kFiTets{{{f, i, e, h},
                                                   {f, i, h, j},
                                                   {f, i, j, g},
                                                   {f, e, i, g}}};
  const std::array<std::array<int, 4>, 4> kEjTets{{{e, h, j, i},
                                                   {e, i, j, g},
                                                   {e, g, j, f},
                                                   {e, f, j, h}}};
  // clang-format on

  // Find direction with best quality.
  double max_quality = -1.0;
  const std::array<std::array<int, 4>, 4>* best_tets = nullptr;
  for (const auto* candidate_tets : {&kGhTets, &kFiTets, &kEjTets}) {
    const double candidate_quality = split_quality(*candidate_tets);
    if (candidate_quality > max_quality) {
      max_quality = candidate_quality;
      best_tets = candidate_tets;
    }
  }
  DRAKE_DEMAND(max_quality > 0.0);
  for (int t = 0; t < 4; ++t) {
    const auto& local_tets = *best_tets;
    tets->emplace_back(
        vertex_indices[local_tets[t][0]], vertex_indices[local_tets[t][1]],
        vertex_indices[local_tets[t][2]], vertex_indices[local_tets[t][3]]);
  }
}

/* Splits the given tetrahedron into 8 tetrahedra which form a complete,
 disjoint decomposition of the original tetrahedron's volume.
 The input tetrahedron defines the indices of the original vertices. An edge
 in the input tetrahedron is described by the two vertices at its ends
 (e.g., (v0, v1), (v1, v3), etc.) The input `edge_map` defines a mapping from
 each edge (vi, vj) to the index of the vertex previously defined at that edge's
 midpoint.

 @param t           The tetrahedron to split.
 @param edge_map    The map from edges to each edge's splitting vertex index.
 @return A vector of 8 tetrahedra.  */
template <typename T>
std::vector<VolumeElement> SplitTetrahedron(
    const VolumeElement& t,
    const std::unordered_map<SortedPair<int>, int>& edge_map,
    const std::vector<Vector3<T>>& p_MVs) {
  using Key = SortedPair<int>;
  std::vector<VolumeElement> tets;
  tets.reserve(8);
  // Indices of the original four vertices.
  const int a(t.vertex(0)), b(t.vertex(1)), c(t.vertex(2)), d(t.vertex(3));
  // Indices of the vertices that split each of the edges.
  const int e(edge_map.at(Key(a, b)));
  const int f(edge_map.at(Key(a, c)));
  const int g(edge_map.at(Key(a, d)));
  const int h(edge_map.at(Key(b, c)));
  const int i(edge_map.at(Key(b, d)));
  const int j(edge_map.at(Key(c, d)));

  //                +Z
  //                 |
  //                 d
  //                 |
  //                 g---j
  //                /|   |
  //               i a---f---c---+Y
  //               |/   /
  //               e---h
  //              /
  //             b
  //            /
  //          +X

  // The four tetrahedra at the corners.
  tets.emplace_back(a, e, f, g);
  tets.emplace_back(b, h, e, i);
  tets.emplace_back(f, h, c, j);
  tets.emplace_back(j, g, i, d);

  SplitOctohedron({e, f, g, h, i, j}, p_MVs, &tets);

  return tets;
}

/* Creates a finer volume mesh approximating a unit sphere (not generally
 applicable to refining other meshes). The refinement is achieved by decomposing
 every tetrahedron into eight new tetrahedron which encompass the same volume.
 The new tetrahedron are formed by splitting the six edges of the original
 tetrahedron in half (adding six new vertices), and building eight new
 tetrahedra topology in that region.

 This algorithm has two interesting properties:

   - It guarantees to introduce no duplicate vertices.
   - If an edge of the input mesh has two vertices that are on the surface of
     the unit sphere, the new vertex that splits that edge will likewise
     lie on the surface of the unit sphere. In other words, with successive
     refinement calls, the mesh's *surface* gets rounder. This property
     precludes the use of this function on arbitrary meshes.

 The position vectors to the new mesh's vertices are measured and expressed in
 the same frame as the input mesh.

 @param mesh            The mesh to refine -- must be a mesh generated either by
                        MakeSphereMeshLevel0() or a previous invocation of
                        RefineUnitSphereMesh().
 @param is_boundary     Classification of vertices in `mesh`. For vertex with
                        index v, `is_boundary[v]` will report true if the vertex
                        lies on the surface of the sphere, or false for the
                        interior.
 @returns  A new mesh (the refined version of the input mesh) and the
 corresponding classifications of the new mesh's vertices.

 @tparam T  The Eigen scalar for the underlying vertex position representation.
 */
template <typename T>
std::pair<VolumeMesh<T>, std::vector<bool>> RefineUnitSphereMesh(
    const VolumeMesh<T>& mesh, const std::vector<bool>& is_boundary) {
  // Initialize the set of new vertices with all of the original vertices; we
  // will grow this set by splitting edges.
  std::vector<Vector3<T>> all_vertices(mesh.vertices());
  std::vector<bool> all_boundary(is_boundary);

  // A map from the indices of two vertices on an edge to the index of the
  // new vertex that sits in the middle of that same edge.
  std::unordered_map<SortedPair<int>, int> edge_vertex_map;
  // An enumeration of all of the edges of a tet.
  const std::vector<std::pair<int, int>> kEdges{{0, 1}, {0, 2}, {0, 3},
                                                {1, 2}, {1, 3}, {2, 3}};
  for (const auto& t : mesh.tetrahedra()) {
    for (const auto& v_pair : kEdges) {
      const SortedPair<int> key{t.vertex(v_pair.first),
                                t.vertex(v_pair.second)};
      if (edge_vertex_map.count(key) == 0) {
        // We haven't already split this edge; compute the vertex and determine
        // its boundary condition.
        const Vector3<T>& p_MA = mesh.vertex(key.first());
        const Vector3<T>& p_MB = mesh.vertex(key.second());
        Vector3<T> p_MV = (p_MA + p_MB) / 2.0;
        int split_index(static_cast<int>(all_vertices.size()));
        const bool split_is_boundary =
            is_boundary[key.first()] && is_boundary[key.second()];
        if (split_is_boundary) p_MV.normalize();

        all_boundary.push_back(split_is_boundary);
        all_vertices.emplace_back(p_MV);
        edge_vertex_map.insert({key, split_index});
      }
    }
  }

  // Now split the tetrahedra
  std::vector<VolumeElement> all_tets;
  // Every input tetrahedron becomes 8 smaller tetrahedra
  all_tets.reserve(mesh.num_elements() * 8);
  for (const auto& t : mesh.tetrahedra()) {
    std::vector<VolumeElement> split_tets =
        SplitTetrahedron(t, edge_vertex_map, all_vertices);
    all_tets.insert(all_tets.end(), split_tets.begin(), split_tets.end());
  }
  return {VolumeMesh<T>(std::move(all_tets), std::move(all_vertices)),
          all_boundary};
}

/* Creates a finer volume mesh approximating a unit sphere (not generally
 applicable to refining other meshes). The refinement is achieved by decomposing
 every tetrahedron face that lies on the sphere boundary into four faces by
 splitting each edge. Four tets are created from those four faces and the
 central vertex to replace the original tetrahedron.

 This algorithm has two interesting properties:

   - It guarantees to introduce no duplicate vertices.
   - All exterior vertices lie on the surface of the unit sphere, regardless of
     refinement level.

 The position vectors to the new mesh's vertices are measured and expressed in
 the same frame as the input mesh.

 @param mesh            The mesh to refine -- must be a mesh generated either by
                        MakeSphereMeshLevel0() or a previous invocation of
                        RefineUnitSphereMeshOnSurface().
 @param center_index    The index of the vertex at the center of the sphere.
 @returns  A new mesh (the refined version of the input mesh) and the
 index of the center vertex in that mesh.
 @pre Every tet in the input mesh includes the origin vertex and has it as its
 _fourth_ vertex.
 @post The returned mesh is a viable mesh for further refinement by this method.

 @tparam T  The Eigen scalar for the underlying vertex position representation.
 */
template <typename T>
std::pair<VolumeMesh<T>, int> RefineUnitSphereMeshOnSurface(
    const VolumeMesh<T>& mesh, int center_index) {
  // Initialize the set of new vertices with all of the original vertices; we
  // will grow this set by splitting edges.
  std::vector<Vector3<T>> all_vertices(mesh.vertices());

  // A map from the indices of two vertices on an edge to the index of the
  // new vertex that sits in the middle of that same edge.
  std::unordered_map<SortedPair<int>, int> edge_vertex_map;
  // An enumeration of the edges of the tets that lie on the surface; we rely on
  // the documented behavior that the level 0 mesh has the origin as the fourth
  // vertex in each tet and preserves this property on all refined tetrahedra.
  const std::vector<std::pair<int, int>> kEdges{{0, 1}, {0, 2}, {1, 2}};
  for (const auto& t : mesh.tetrahedra()) {
    DRAKE_DEMAND(t.vertex(3) == center_index);
    for (const auto& v_pair : kEdges) {
      const SortedPair<int> key{t.vertex(v_pair.first),
                                t.vertex(v_pair.second)};
      // TODO(SeanCurtis-TRI): Refactor edge refinement into a single method.
      if (edge_vertex_map.count(key) == 0) {
        // We haven't already split this edge; compute the vertex and determine
        // its boundary condition.
        const Vector3<T>& p_MA = mesh.vertex(key.first());
        const Vector3<T>& p_MB = mesh.vertex(key.second());
        Vector3<T> p_MV = (p_MA + p_MB) / 2.0;
        // "Inflate" the surface vertex to the unit sphere surface.
        p_MV.normalize();
        int split_index = static_cast<int>(all_vertices.size());

        all_vertices.emplace_back(p_MV);
        edge_vertex_map.insert({key, split_index});
      }
    }
  }

  // Now split the tetrahedra.
  std::vector<VolumeElement> all_tets;
  // Every input tetrahedron becomes 4 smaller tetrahedra.
  all_tets.reserve(mesh.num_elements() * 4);

  /*
                  │                     Vertex v0 represents the center of the
                  ● v0                  sphere. Vertices a, b, and c are the
                  │                     vertices of the original tet that lie on
                  │                     the sphere surface. The vertices d, e,
                  │                     and f are:
                  │                        d ≜ the split vertex between b & c
                  │                        e ≜ the split vertex between a & b
                  │ c   d     b            f ≜ the split vertex between a & c
                  ●────◯─────●───       The four new tets we create are:
                 ╱                         (a, e, f, v0)
                ╱      ◯ e                 (b, d, e, v0)
               ◯ f                         (c, f, d, v0)
              ╱                            (e, d, f, v0)
             ● a
            ╱
  */
  const int v0 = center_index;
  for (const auto& t : mesh.tetrahedra()) {
    // We've already confirmed that vertex 3 of every tetrahedron is the center
    // vertex.
    const int a = t.vertex(0);
    const int b = t.vertex(1);
    const int c = t.vertex(2);
    const int d = edge_vertex_map.at(SortedPair<int>{b, c});
    const int e = edge_vertex_map.at(SortedPair<int>{a, b});
    const int f = edge_vertex_map.at(SortedPair<int>{a, c});
    all_tets.emplace_back(a, e, f, v0);
    all_tets.emplace_back(b, d, e, v0);
    all_tets.emplace_back(c, f, d, v0);
    all_tets.emplace_back(e, d, f, v0);
  }
  return {VolumeMesh<T>(std::move(all_tets), std::move(all_vertices)),
          center_index};
}

// TODO(SeanCurtis-TRI): Consider splitting this into two different functions.
//  This could ease profiling because it is apparent what type of mesh is being
//  generated by the function invocation.
/* This method implements a variant of the generator described in
 [Everett, 1997]. It is based on a recursive refinement of an initial
 (refinement_level = 0) coarse mesh representation of the unit sphere. The
 initial mesh discretizes an octahedron with its six vertices on the
 surface of the unit sphere and a seventh vertex at the origin to form a
 volume tessellation consisting of eight tetrahedra. At each refinement
 level, each tetrahedron is split into eight new tetrahedra by splitting
 each edge in half. When splitting an edge formed by vertices on the
 surface of the sphere, the newly created vertex is projected back onto the
 surface of the sphere.
 See [Jakub Velímský, 2010] for additional implementation details and a
 series of very useful schematics.

 @throws std::exception if refinement_level is negative.

 [Everett, 1997]  Everett, M.E., 1997. A three-dimensional spherical mesh
 generator. Geophysical Journal International, 130(1), pp.193-200.
 [Jakub Velímský, 2010] GESTIKULATOR Generator of a tetrahedral mesh on a
 sphere. Department of Geophysics, Charles University in Prague.
 */
template <typename T>
VolumeMesh<T> MakeUnitSphereMesh(int refinement_level,
                                 TessellationStrategy strategy) {
  DRAKE_THROW_UNLESS(refinement_level >= 0);
  auto [mesh, is_boundary] = MakeSphereMeshLevel0<T>();

  switch (strategy) {
    case TessellationStrategy::kSingleInteriorVertex: {
      int center_index = -1;
      for (int i = 0; i < static_cast<int>(is_boundary.size()); ++i) {
        if (is_boundary[i] == false) {
          // There should be only *one* vertex not on the boundary of the level
          // 0 mesh -- the center vertex.
          center_index = i;
          break;
        }
      }
      DRAKE_DEMAND(center_index >= 0);
      for (int level = 1; level <= refinement_level; ++level) {
        std::tie(mesh, center_index) =
            RefineUnitSphereMeshOnSurface<T>(mesh, center_index);
        DRAKE_DEMAND(center_index == 0);
      }
      break;
    }
    case TessellationStrategy::kDenseInteriorVertices: {
      for (int level = 1; level <= refinement_level; ++level) {
        std::tie(mesh, is_boundary) =
            RefineUnitSphereMesh<T>(mesh, is_boundary);
        DRAKE_DEMAND(mesh.vertices().size() == is_boundary.size());
      }
      break;
    }
    default:
      DRAKE_UNREACHABLE();
  }
  return mesh;
}

/* Creates a volume mesh for the given `sphere`; the level of tessellation is
 guided by the `resolution_hint` parameter.

 `resolution_hint` influences the resolution of the mesh. Smaller values create
 higher-resolution meshes with smaller tetrahedra. The resolution hint is
 interpreted as an edge length and the sphere is subdivided to guarantee that
 edge lengths along the *equator* of the sphere will be less than or equal to
 that edge length.

 The resolution of the final mesh will change discontinuously. Small changes to
 `resolution_hint` will likely produce the same mesh. However, in the current
 implementation, cutting `resolution_hint` in half _will_ increase the number of
 tetrahdra.

 Ultimately, successively smaller values of `resolution_hint` will no longer
 change the output mesh. This algorithm limits the maximum resolution to prevent
 accidental instantiation of impractical meshes. At its maximum resolution,
 the sphere's surface will have approximately half a million triangles (with
 the corresponding number of interior tetrahedra based on the given tessellation
 strategy). Similarly, for arbitrarily large values of `resolution_hint`, the
 coarsest possible mesh is a tessellated octahedron.

 @param sphere              The sphere for which a mesh is created.
 @param resolution_hint     The positive characteristic edge length for the
                            sphere (same units of length as `sphere.radius()`).
                            The coarsest possible mesh (an octahedron) is
                            guaranteed for any value of `resolution_hint`
                            greater than or equal to the `sphere`'s diameter.
 @param strategy            The strategy to use to tessellate the sphere. See
                            TesselationStrategy for details.
 @return The volume mesh for the given sphere.
 @tparam T  The Eigen-compatible scalar for representing the mesh vertex
            positions.
 */
template <typename T>
VolumeMesh<T> MakeSphereVolumeMesh(const Sphere& sphere,
                                   double resolution_hint,
                                   TessellationStrategy strategy) {
  /*
    The volume mesh is formed by successively refining an octahedron. At the
    equator, that means we go from 4 edges, to 8 edges, to 16 edges, etc.,
    doubling at each level of refinement. These edges are simply chords across
    fixed-length arcs of the sphere. Based on that we can easily compute the
    length of the chord and bound it by resolution_hint.

              /|
           r / |
            /  |
           <θ  | e
            \  |
           r \ |
              \|

     The length of e = 2⋅r⋅sin(θ/2). Solving for θ we get: θ = 2⋅sin⁻¹(e/2⋅r).
     We can relate θ with the refinement level ℒ.

       θ = 2π / 4⋅2ᴸ
         = π / 2⋅2ᴸ
         = π / 2ᴸ⁺¹

     Substituting for θ, we get:

       π / 2ᴸ⁺¹ = 2⋅sin⁻¹(e/2⋅r)
       2ᴸ⁺¹ = π / 2⋅sin⁻¹(e/2⋅r)
       ℒ + 1 = log₂(π / 2⋅sin⁻¹(e/2⋅r))
       ℒ = log₂(π / 2⋅sin⁻¹(e/2⋅r)) - 1
       ℒ = ⌈log₂(π / sin⁻¹(e/2⋅r))⌉ - 2
   */
  DRAKE_DEMAND(resolution_hint > 0.0);
  const double r = sphere.radius();
  // Make sure the arcsin doesn't blow up.
  const double edge_length = std::min(resolution_hint, 2.0 * r);
  const int L = std::max(
      0,
      static_cast<int>(
          std::ceil(std::log2(M_PI / std::asin(edge_length / (2.0 * r)))) - 2));
  // TODO(SeanCurtis-TRI): Consider pushing the radius into the sphere creation
  //  so that copying vertices and tets is no longer necessary.

  // Note: With refinement L = 8, we'd get 8 ^(8 + 1) = 134M tetrahedra for a
  // dense mesh and 8 * 4^8 = 500K tetrahedra for a sparse mesh, satisfying the
  // promise that we won't produce unlimited tetrahedra.
  VolumeMesh<T> unit_mesh = MakeUnitSphereMesh<T>(std::min(L, 8), strategy);
  std::vector<Vector3<T>> vertices;
  vertices.reserve(unit_mesh.vertices().size());
  for (auto& v : unit_mesh.vertices()) {
    vertices.emplace_back(v * r);
  }
  std::vector<VolumeElement> elements(unit_mesh.tetrahedra());
  return VolumeMesh<T>(std::move(elements), std::move(vertices));
}

/* Creates a surface mesh for the given `sphere`; the level of tessellation
 is guided by the `resolution_hint` parameter in the same way as
 MakeSphereVolumeMesh.

 @param sphere              The sphere for which a surface mesh is created.
 @param resolution_hint     The positive characteristic edge length for the
                            sphere (same units of length as `sphere.radius()`).
                            The coarsest possible mesh (an octahedron) is
                            guaranteed for any value of `resolution_hint`
                            greater than or equal to the `sphere`'s diameter.
 @return The triangulated surface mesh for the given sphere.
 @tparam T  The Eigen-compatible scalar for representing the mesh vertex
            positions.
 */
template <typename T>
TriangleSurfaceMesh<T> MakeSphereSurfaceMesh(const Sphere& sphere,
                                             double resolution_hint) {
  DRAKE_DEMAND(resolution_hint > 0.0);
  return ConvertVolumeToSurfaceMesh<T>(MakeSphereVolumeMesh<T>(
      sphere, resolution_hint, TessellationStrategy::kSingleInteriorVertex));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
