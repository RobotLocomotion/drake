#pragma once

#include <limits>
#include <map>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(DamrongGuoy) Consider moving FeatureNormalSet into its own file if it
//  is useful for other applications.

// %FeatureNormalSet provides a certain kind of outward normal vectors at
// vertices and edges of a triangle surface mesh. The normal at a vertex is
// the angle weighted average of face normals of triangles sharing the vertex.
// The normal at an edge is the equal-weight average of face normals of two
// triangles sharing the edge.
//
// The following paper shows that this kind of normal is suitable for the
// inside-outside test of a point closest to a vertex or an edge, and other
// kinds of normal vectors may give incorrect result.
//
// J.A. Baerentzen; H. Aanaes. Signed distance computation using the angle
// weighted pseudonormal. IEEE Transactions on Visualization and Computer
// Graphics (Volume: 11, Issue: 3, May-June 2005).
class FeatureNormalSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FeatureNormalSet);

  // Computes and stores the normals at vertices and edges of the given
  // surface mesh.
  explicit FeatureNormalSet(const TriangleSurfaceMesh<double>& mesh_M);

  // Returns the normal at a vertex `v` as the angle weighted average of face
  // normals of triangles sharing the vertex. The weight of a triangle is the
  // angle at vertex `v` in that triangle.
  //
  // The returned normal vector is expressed in the mesh's frame.
  //
  // @param v  the vertex index into the mesh
  //
  // @pre 0 <= v < mesh_M.num_vertices()
  Vector3<double> vertex_normal(int v) const { return vertex_normals_.at(v); }

  // Returns the normal at an edge `uv` as the average normal from the two
  // triangles sharing the edge. Both triangles have equal weight.
  //
  // The returned normal vector is expressed in the mesh's frame.
  //
  // @param uv   the SortedPair<int>{u, v} of the edge between vertex
  //             index u and vertex index v
  //
  // @pre 0 <= u,v < mesh_M.num_vertices()
  // @pre The edge `uv` is in the mesh.
  Vector3<double> edge_normal(const SortedPair<int>& uv) const {
    return edge_normals_.at(uv);
  }

 private:
  std::vector<Vector3<double>> vertex_normals_{};
  std::map<SortedPair<int>, Vector3<double>> edge_normals_{};
};

// %SquaredDistanceToTriangle stores information about squared distance from
// a query point Q to a triangle for the inside-outside test.
struct SquaredDistanceToTriangle {
  double squared_distance{std::numeric_limits<double>::infinity()};
  // The point in the triangle closest to the query point Q.
  Vector3<double> closest_point{};

  // Classify the projection Q' of the query point Q onto the plane of the
  // triangle.
  enum class Projection {
    // Q' is in the triangle. It could be anywhere in the triangle including
    // its edges and vertices.  The `closest_point` is Q'.  We can use the
    // face normal of the triangle for the inside-outside test.
    //                                  A
    //                                🮠│
    //                             🮠   │
    //                          🮠      │
    //                       🮠   Q'    │
    //                   B ┄┄┄┄┄┄┄┄┄┄┄┄┄C
    kInside,

    // Q' is outside the triangle and nearest to a point in an edge
    // excluding its vertices. The `closest_point` is the projection of Q'
    // onto that edge. We can use the edge normal (the equal-weight average
    // of face normals of two triangles sharing the edge) for the
    // inside-outside test. In the picture below, the area between the
    // two rays is the region of points that are closest to edge BC (Voronoi
    // region of the edge).
    //                                  A
    //                                🮠│
    //                             🮠   │
    //                          🮠      │
    //                       🮠         │
    //                   B ┄┄┄┄┄┄┄┄┄┄┄┄┄C
    //                   ↓              ↓
    //                   ↓       Q'     ↓
    //                   ↓              ↓
    kOutsideNearEdge,

    // Q' is outside the triangle and nearest to a vertex of the triangle.
    // The `closest_point` is that vertex. We can use the vertex normal (the
    // angle-weighted average of face normals of triangles sharing the vertex)
    // for the inside-outside test. In the picture below, the area between
    // the two rays is the region of points that are closest to
    // vertex B (Voronoi region of the vertex).
    //                                   A
    //                                🮠│
    //         ↖                   🮠   │
    //            ↖             🮠      │
    //               ↖       🮠         │
    //                   B ┄┄┄┄┄┄┄┄┄┄┄┄┄C
    //       Q'          ↓
    //                   ↓
    //                   ↓
    kOutsideNearVertex
  } location{};

  // The meaning of v depends on `location`:
  // - kInside: v is zero. The closest point is in the triangle.
  // - kOutsideNearEdge: The closest point is in the edge between the
  //                     triangle's v-th vertex and (v+1)%3-th vertex;
  //                     0 <= v < 3.
  // - kOutsideNearVertex: The closest point is at the triangle's v-th vertex;
  //                       0 <= v < 3.
  int v{};

  auto operator<=>(const SquaredDistanceToTriangle&) const = default;
};

// TODO(DamrongGuoy): Consider consolidating this function with the one in
//  the anonymous namespace of calc_distance_to_surface_mesh.cc. That one
//  gives the squared distance without the closest point and the classification
//  of the query point.

SquaredDistanceToTriangle CalcSquaredDistanceToTriangle(
    const Vector3<double>& p_MQ, int triangle_index,
    const TriangleSurfaceMesh<double>& mesh_M);

struct SignedDistanceToSurfaceMesh {
  double signed_distance;
  Vector3<double> nearest_point;
  Vector3<double> gradient;

  auto operator<=>(const SignedDistanceToSurfaceMesh&) const = default;
};

// Calculates the signed distance, the nearest point, and the signed-distance
// gradient from the query point Q to the surface mesh. It accelerates the
// computation using BVH. It determines the sign using the given
// FeatureNormalSet of the mesh.
//
// @param p_MQ  position of the query point expressed in frame M of the
//              surface mesh.
// @param mesh_M   the surface mesh expressed in frame M.
// @param bvh_M    the BVH of the surface mesh, expressed in frame M.
// @param mesh_normal_M  provides angle-weighted average normals at vertices
//                       and equal-weight average normals at edges of the
//                       surface mesh, expressed in frame M.
//
// @pre  The surface mesh is watertight and a closed manifold. Otherwise, it
// might return incorrect signs and gradients.
//
// @note If p_MQ is equally far from multiple faces, the nearest point and
// the gradient are selected arbitrarily from those faces.
//
// @note If p_MQ is on the surface, the returned signed distance is zero,
// the nearest point is p_MQ itself, and the gradient is in the conical hull
// of outward face normals of triangles that contain p_MQ. (There are two such
// triangles if p_MQ is in an edge. There are multiple such triangles
// if p_MQ is at a vertex.)
SignedDistanceToSurfaceMesh CalcSignedDistanceToSurfaceMesh(
    const Vector3<double>& p_MQ, const TriangleSurfaceMesh<double>& mesh_M,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_M,
    const FeatureNormalSet& mesh_normal_M);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
