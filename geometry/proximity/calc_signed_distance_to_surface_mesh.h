#pragma once

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

// TODO(DamrongGuoy) Consider moving VertexEdgeNormal into its own file if it
//  is useful for other applications.

// %VertexEdgeNormal provides a certain kind of outward normal vectors at
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
class VertexEdgeNormal {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VertexEdgeNormal);

  // Computes and stores the normals at vertices and edges of the given
  // surface mesh.
  explicit VertexEdgeNormal(const TriangleSurfaceMesh<double>& mesh_M);

  // Returns the normal at a vertex `v` as the angle weighted average of face
  // normals of triangles sharing the vertex. The weight of a triangle is the
  // angle at vertex `v` in that triangle.
  //
  // The returned normal vector is expressed in the mesh's frame.
  //
  // @param v  the vertex index into the mesh
  //
  // @pre 0 <= v < mesh_M.num_vertices()
  Vector3<double> vertex_normal(int v) const { return vertex_normal_.at(v); }

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
    return edge_normal_.at(uv);
  }

 private:
  std::vector<Vector3<double>> vertex_normal_{};
  std::map<SortedPair<int>, Vector3<double>> edge_normal_{};
};

struct SquaredDistanceToTriangle {
  double squared_distance;
  Vector3<double> closest_point{};
  enum class Location { kTriangle, kEdge, kVertex } location;
  // The meaning of v depends on location:
  // - kTriangle: v is zero. The closest point is in the triangle.
  // - kEdge: The closest point is in the edge between the triangle's
  //          v-th vertex and (v+1)%3-th vertex; 0 <= v < 3.
  // - kVertex: The closest point is at the triangle's v-th vertex; 0 <= v < 3.
  int v;

  auto operator<=>(const SquaredDistanceToTriangle&) const = default;
};

// TODO(DamrongGuoy): Consider consolidating this function with the one in
//  the anonymous namespace of calc_distance_to_surface_mesh.cc. That one
//  gives the squared distance without the closest point and its location.

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
// VertexEdgeNormal of the mesh.
//
// @param p_MQ  position of the query point expressed in frame M of the
//              surface mesh.
// @param mesh_M   the surface mesh expressed in frame M.
// @param bvh_M    the BVH of the surface mesh, expressed in frame M.
// @param mesh_normal_M  provides angle weighted normals at vertices and
//                       edges of the surface mesh, expressed in frame M.
SignedDistanceToSurfaceMesh CalcSignedDistanceToSurfaceMesh(
    const Vector3<double>& p_MQ, const TriangleSurfaceMesh<double>& mesh_M,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_M,
    const VertexEdgeNormal& mesh_normal_M);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
