#pragma once

#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_assert.h"
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

// TODO(DamrongGuoy): Add to the document of ComputeSignedDistanceToPoint()
//  in QueryObject the discussion that extremely sharp features can make
//  FeatureNormalSet throw exceptions.

/* %FeatureNormalSet provides outward normal vectors at vertices and edges of a
 triangle surface mesh. The normal at a vertex is the angle-weighted average of
 the face normals of the triangles sharing the vertex. The normal at an edge is
 the equal-weight average of the face normals of the two triangles sharing the
 edge.

 The following paper shows that normals computed in this way are most suitable
 for the inside-outside test of a point closest to a vertex or an edge.

 J.A. Baerentzen; H. Aanaes. Signed distance computation using the angle
 weighted pseudonormal. IEEE Transactions on Visualization and Computer
 Graphics (Volume: 11, Issue: 3, May-June 2005).  */
class FeatureNormalSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FeatureNormalSet);

  // Computes the normals at the vertices and the edges of the given surface
  // mesh expressed in frame M.
  //
  // @pre  The mesh is watertight and a closed manifold. Otherwise, the
  //       computed normals are incorrect.
  //
  // @returns the computed feature normals or an error message if the mesh has
  // extremely sharp features. They make calculation of the vertex normals and
  // edge normals too sensitive to numerical rounding.
  //     Think of the simplest possible knife model of two adjacent triangles
  // sharing a single edge. The two triangles would have a small angle between
  // them, creating a "sharp edge".  If the edge is too sharp, the mesh will be
  // rejected.  We have a generous tolerance -- the angle can be less than a
  // degree -- but it would be better to stay well away from impractically sharp
  // edges.
  //     There is an analogous situation with vertices, where a vertex
  // creates a pointy, needle-like region on the mesh. The skinny "needles"
  // have the same problem as the thin knife edges. Presence of these
  // "needles" may lead to the mesh being rejected.
  static std::variant<FeatureNormalSet, std::string> MaybeCreate(
      const TriangleSurfaceMesh<double>& mesh_M);

  // Returns the normal at a vertex `v` as the angle-weighted average of face
  // normals of triangles sharing the vertex. The weight of a triangle is the
  // angle at vertex `v` in that triangle.
  //
  // The returned normal vector is expressed in the mesh's frame.
  //
  // @param v  the vertex index into the mesh
  //
  // @throws std::exception if v < 0 or v >= number of vertices in the mesh
  const Vector3<double>& vertex_normal(int v) const {
    DRAKE_THROW_UNLESS(0 <= v && v < ssize(vertex_normals_));
    return vertex_normals_[v];
  }

  // Returns the normal at an edge `uv` as the average normal from the two
  // triangles sharing the edge. Both triangles have equal weight.
  //
  // The returned normal vector is expressed in the mesh's frame.
  //
  // @param uv   the edge between vertices u and v, as represented by the
  //             sorted indices of vertices.
  //
  // @throws std::exception if the edge `uv` is not in the mesh.
  const Vector3<double>& edge_normal(const SortedPair<int>& uv) const {
    auto it = edge_normals_.find(uv);
    DRAKE_THROW_UNLESS(it != edge_normals_.end());
    return it->second;
  }

 private:
  FeatureNormalSet(
      std::vector<Vector3<double>>&& vertex_normals,
      std::unordered_map<SortedPair<int>, Vector3<double>>&& edge_normals)
      : vertex_normals_(std::move(vertex_normals)),
        edge_normals_(std::move(edge_normals)) {}

  std::vector<Vector3<double>> vertex_normals_{};
  std::unordered_map<SortedPair<int>, Vector3<double>> edge_normals_{};
};

/* %SquaredDistanceToTriangle stores information about squared distance from
 a query point Q to a triangle for the inside-outside test.  */
struct SquaredDistanceToTriangle {
  double squared_distance{std::numeric_limits<double>::infinity()};
  // The point in the triangle closest to the query point Q.
  Vector3<double> closest_point;
  // The normal at the closest feature (vertex, edge, or triangle).
  Vector3<double> feature_normal;
};

/* Calculates the squared distance and the closest point from the query point
 Q to a triangle in a mesh.

 It also provides the normal at the closest point suitable for the
 inside-outside test (see FeatureNormalSet) as follows:
 - return the vertex normal if the closest point is at a vertex, otherwise
 - return the edge normal if the closest point is in an edge, otherwise
 - return the face normal of the triangle.  */
SquaredDistanceToTriangle CalcSquaredDistanceToTriangle(
    const Vector3<double>& p_MQ, int triangle_index,
    const TriangleSurfaceMesh<double>& mesh_M,
    const FeatureNormalSet& normal_set_M);

struct SignedDistanceToSurfaceMesh {
  double signed_distance;
  Vector3<double> nearest_point;
  // The gradient is not continuous when the query point is at a vertex or in
  // an edge shared by triangles with different face normals.
  // The gradient is also discontinuous at the query point with multiple
  // nearest points.
  Vector3<double> gradient;
};

// TODO(DamrongGuoy): Explain the precondition of the meshes for
//  CalcSignedDistanceToSurfaceMesh in ComputeSignedDistanceToPoint()
//  in QueryObject.

/* Calculates the signed distance, the nearest point, and the signed-distance
 gradient from the query point Q to the surface mesh. It accelerates the
 computation using a BVH. It determines the sign using the given
 FeatureNormalSet of the mesh.

 @param p_MQ  position of the query point expressed in frame M of the
              surface mesh.
 @param mesh_M   the surface mesh expressed in frame M.
 @param bvh_M    the BVH of the surface mesh, expressed in frame M.
 @param feature_normals_M  provides angle-weighted average normals at
                           vertices and equal-weight average normals at
                           edges of the surface mesh, expressed in frame M.

 @pre  The surface mesh is a closed manifold without duplicate vertices or
 self-intersection, and every triangle's face winding gives an outward-pointing
 face normal. Non-compliant meshes will introduce regions in which the query
 point will report the wrong sign (and, therefore, the wrong gradient) due to a
 misclassification of being inside or outside. This leads to discontinuities in
 the distance field across the boundaries of these regions; the distance sign
 will flip while the magnitude of the distance value is arbitrarily far away
 from zero. For open meshes, the same principle holds. The open mesh, which has
 no true concept of "inside", will nevertheless report some query points as
 being inside.

 @note If p_MQ is on the surface, the returned signed distance is zero,
 the nearest point is p_MQ itself, and the gradient is the normal
 at the vertex, edge or triangle on which p_MQ lies.

 @note If p_MQ has multiple nearest points, the returned nearest point
 and the gradient are chosen and computed from one of them. The selection
 is deterministic but arbitrary because we use the given BVH to select the
 first closest point. The search order depends on the structure of
 the BVH.

 The following table characterizes all possible cases. The sign is positive,
 negative, or zero when the query point is outside, inside, or on the
 surface respectively. The nearest point N could lie in a triangle, an edge,
 or a vertex. The gradient depends on the sign of the distance.

  |   sign   |  location of  |    gradient    |
  |          | nearest point |                |
  | :------: | :-----------: | :------------: |
  | positive |   triangle    | (Q-N) / ‖Q-N‖  |
  | positive |     edge      | (Q-N) / ‖Q-N‖  |
  | positive |    vertex     | (Q-N) / ‖Q-N‖  |
  | negative |   triangle    | (N-Q) / ‖N-Q‖  |
  | negative |     edge      | (N-Q) / ‖N-Q‖  |
  | negative |    vertex     | (N-Q) / ‖N-Q‖  |
  |   zero   |   triangle    |  face normal   |
  |   zero   |     edge      |  edge normal   |
  |   zero   |    vertex     | vertex normal  |
  __*Table 1*__: Possible cases of signed distances and gradients according
  to the location of the nearest point.  */
SignedDistanceToSurfaceMesh CalcSignedDistanceToSurfaceMesh(
    const Vector3<double>& p_MQ, const TriangleSurfaceMesh<double>& mesh_M,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_M,
    const FeatureNormalSet& feature_normals_M);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
