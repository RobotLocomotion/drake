#include "drake/geometry/proximity/calc_signed_distance_to_surface_mesh.h"

#include <algorithm>
#include <array>
#include <limits>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/distance_to_point_callback.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;
using math::RigidTransformd;

namespace {

class BvhVisitor {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BvhVisitor);

  static void CalcSquaredDistance(
      const Vector3d& p_MQ, const TriangleSurfaceMesh<double>& mesh_M,
      const Bvh<Obb, TriangleSurfaceMesh<double>>::NodeType& node_M,
      int* triangle_index, SquaredDistanceToTriangle* squared_distance) {
    BvhVisitor visitor{p_MQ, mesh_M};
    visitor.Visit(node_M);
    *triangle_index = visitor.closest_triangle_index_;
    *squared_distance = visitor.closest_info_;
    return;
  }

 private:
  BvhVisitor(const Vector3d& p_MQ, const TriangleSurfaceMesh<double>& mesh_M)
      : p_MQ_{p_MQ}, mesh_M_{mesh_M} {}

  void Visit(const Bvh<Obb, TriangleSurfaceMesh<double>>::NodeType& node_M) {
    if (node_M.is_leaf()) {
      for (int i = 0; i < node_M.num_element_indices(); ++i) {
        const int triangle_index = node_M.element_index(i);
        const SquaredDistanceToTriangle squared_distance =
            CalcSquaredDistanceToTriangle(p_MQ_, triangle_index, mesh_M_);
        if (squared_distance.squared_distance <
            closest_info_.squared_distance) {
          closest_info_ = squared_distance;
          closest_triangle_index_ = triangle_index;
        }
      }
      return;
    }

    // The rest of this function is for the subtree of the internal node rooted
    // at node_M with its bounding box B, expressed in frame B.
    const RigidTransformd& X_MB = node_M.bv().pose();
    const Vector3d p_BQ = X_MB.inverse() * p_MQ_;
    // Cb and grad_B are the closest point and signed-distance gradient to the
    // query point Q with respect to the surface of the bounding box B.
    const auto [p_BCb, grad_B, _] =
        point_distance::DistanceToPoint<double>::ComputeDistanceToBox<3>(
            node_M.bv().half_width(), p_BQ);
    // phi_BQ is the signed distance of Q from the bounding box B.
    const double phi_BQ = grad_B.dot(p_BQ - p_BCb);

    // Check for possible pruning.
    if (phi_BQ > 0) {
      // The query point is outside, so we can get the lower bound.
      const double squared_distance_from_node = phi_BQ * phi_BQ;
      // Use the lower bound to possibly prune this subtree.
      if (squared_distance_from_node > closest_info_.squared_distance) {
        return;
      }
    }

    // We couldn't prune the subtree, recursively search both children.
    this->Visit(node_M.left());
    this->Visit(node_M.right());
  }

 private:
  const Vector3<double>& p_MQ_;
  const TriangleSurfaceMesh<double>& mesh_M_;

  int closest_triangle_index_{};
  SquaredDistanceToTriangle closest_info_{
      .squared_distance = std::numeric_limits<double>::infinity()};
};

}  // namespace

FeatureNormalSet::FeatureNormalSet(const TriangleSurfaceMesh<double>& mesh_M) {
  vertex_normals_.resize(mesh_M.num_vertices(), Vector3d::Zero());
  const std::vector<Vector3d>& vertices = mesh_M.vertices();
  // Accumulate data from the mesh. They are not normal vectors yet. We will
  // normalize them afterward.
  for (int f = 0; f < mesh_M.num_triangles(); ++f) {
    const Vector3d& triangle_normal = mesh_M.face_normal(f);
    const int v[3] = {mesh_M.triangles()[f].vertex(0),
                      mesh_M.triangles()[f].vertex(1),
                      mesh_M.triangles()[f].vertex(2)};
    const Vector3d unit_edge_vector[3] = {
        (vertices[v[1]] - vertices[v[0]]).stableNormalized(),
        (vertices[v[2]] - vertices[v[1]]).stableNormalized(),
        (vertices[v[0]] - vertices[v[2]]).stableNormalized()};
    // Accumulate angle*normal for each vertex of the triangle.
    for (int i = 0; i < 3; ++i) {
      const double angle =
          std::acos(unit_edge_vector[i].dot(-unit_edge_vector[(i + 2) % 3]));
      vertex_normals_[v[i]] += angle * triangle_normal;
    }
    // Accumulate normal for each edge of the triangle.
    for (int i = 0; i < 3; ++i) {
      const auto edge = MakeSortedPair(v[i], v[(i + 1) % 3]);
      auto it = edge_normals_.find(edge);
      if (it == edge_normals_.end()) {
        edge_normals_[edge] = triangle_normal;
      } else {
        it->second += triangle_normal;
      }
    }
  }
  for (auto& v_normal : vertex_normals_) {
    v_normal.stableNormalize();
  }
  for (auto& [_, e_normal] : edge_normals_) {
    e_normal.stableNormalize();
  }
}

SquaredDistanceToTriangle CalcSquaredDistanceToTriangle(
    const Vector3<double>& p_MQ, int triangle_index,
    const TriangleSurfaceMesh<double>& mesh_M) {
  Vector3d b_Q = mesh_M.CalcBarycentric(p_MQ, triangle_index);
  if (b_Q(0) >= 0 && b_Q(1) >= 0 && b_Q(2) >= 0) {
    // The projection of Q on the plane of the triangle is in the triangle.
    const Vector3d p_MN =
        mesh_M.CalcCartesianFromBarycentric(triangle_index, b_Q);
    return {(p_MQ - p_MN).squaredNorm(), p_MN,
            SquaredDistanceToTriangle::Location::kTriangle, 0};
  }

  const SurfaceTriangle& triangle = mesh_M.triangles()[triangle_index];
  // Vertex indices into the mesh.
  std::array<int, 3> v = {triangle.vertex(0), triangle.vertex(1),
                          triangle.vertex(2)};
  std::array<Vector3d, 3> p_MV = {mesh_M.vertex(v[0]), mesh_M.vertex(v[1]),
                                  mesh_M.vertex(v[2])};

  // The projection is outside the triangle, so the closest point is
  // in an edge (line segment) or at a vertex.
  SquaredDistanceToTriangle info{.squared_distance =
                                     std::numeric_limits<double>::infinity()};
  // Iterate over three edges, call each edge AB.
  for (int i = 0; i < 3; ++i) {
    const Vector3d p_MA = p_MV[i];
    const Vector3d p_MB = p_MV[(i + 1) % 3];
    const Vector3d p_AB_M = p_MB - p_MA;
    // t = 0 when the projection is at A, and t = 1 at B.
    const double t = p_AB_M.dot(p_MQ - p_MA) / p_AB_M.squaredNorm();
    // N is the nearest point in the line segment.
    Vector3d p_MN;
    SquaredDistanceToTriangle::Location location;
    int index;
    if (t <= 0) {
      p_MN = p_MA;
      location = SquaredDistanceToTriangle::Location::kVertex;
      index = i;
    } else if (t >= 1) {
      p_MN = p_MB;
      location = SquaredDistanceToTriangle::Location::kVertex;
      index = (i + 1) % 3;
    } else {
      p_MN = (1.0 - t) * p_MA + t * p_MB;
      location = SquaredDistanceToTriangle::Location::kEdge;
      index = i;
    }

    const double d = (p_MQ - p_MN).squaredNorm();
    if (d < info.squared_distance) {
      info = SquaredDistanceToTriangle({d, p_MN, location, index});
    }
  }
  return info;
}

SignedDistanceToSurfaceMesh CalcSignedDistanceToSurfaceMesh(
    const Vector3<double>& p_MQ, const TriangleSurfaceMesh<double>& mesh_M,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_M,
    const FeatureNormalSet& mesh_normal_M) {
  int tri_index;
  SquaredDistanceToTriangle closest;
  BvhVisitor::CalcSquaredDistance(p_MQ, mesh_M, bvh_M.root_node(), &tri_index,
                                  &closest);

  const SurfaceTriangle& tri = mesh_M.triangles().at(tri_index);
  Vector3d normal_M;
  {
    const int v = closest.v;
    switch (closest.location) {
      case SquaredDistanceToTriangle::Location::kTriangle:
        normal_M = mesh_M.face_normal(tri_index);
        break;
      case SquaredDistanceToTriangle::Location::kEdge:
        normal_M =
            mesh_normal_M.edge_normal({tri.vertex(v), tri.vertex((v + 1) % 3)});
        break;
      case SquaredDistanceToTriangle::Location::kVertex:
        normal_M = mesh_normal_M.vertex_normal(tri.vertex(v));
    }
  }
  // N is the nearest point.
  const Vector3d p_MN = closest.closest_point;
  const Vector3d p_NQ_M = p_MQ - p_MN;
  const double sign = p_NQ_M.dot(normal_M) >= 0 ? 1 : -1;
  const double unsigned_distance = std::sqrt(closest.squared_distance);
  return {
      .signed_distance = sign * unsigned_distance,
      .nearest_point = p_MN,
      .gradient = (p_NQ_M.isZero()) ? normal_M : sign * p_NQ_M.normalized()};
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
