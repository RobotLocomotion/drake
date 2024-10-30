#include "drake/geometry/proximity/calc_signed_distance_to_surface_mesh.h"

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/distance_to_point_callback.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;
using math::RigidTransformd;

namespace {

// TODO(DamrongGuoy): Consider the techniques in the following paper to speed
//  up the search for closest features:
//  S. Curtis; R. Tamstorf; D. Manocha. Fast collision detection for
//  deformable models using representative-triangles. I3D '08: Proceedings of
//  the 2008 symposium on Interactive 3D graphics and games.

class BvhVisitor {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BvhVisitor);

  static SquaredDistanceToTriangle CalcSquaredDistance(
      const Vector3d& p_MQ, const TriangleSurfaceMesh<double>& mesh_M,
      const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_M,
      const FeatureNormalSet& normal_set_M) {
    BvhVisitor visitor{p_MQ, mesh_M, normal_set_M};
    visitor.Visit(bvh_M.root_node());
    return visitor.closest_info_;
  }

 private:
  BvhVisitor(const Vector3d& p_MQ, const TriangleSurfaceMesh<double>& mesh_M,
             const FeatureNormalSet& normal_set_M)
      : p_MQ_{p_MQ}, mesh_M_{mesh_M}, normal_set_M_{normal_set_M} {}

  void Visit(const Bvh<Obb, TriangleSurfaceMesh<double>>::NodeType& node_M) {
    // Evaluate distance to this node's bounding box B, expressed in frame B.
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
      // The query point is outside box B, so we can get the lower bound.
      const double squared_distance_from_node = phi_BQ * phi_BQ;
      // Use the lower bound to possibly prune this subtree.
      if (squared_distance_from_node >= closest_info_.squared_distance) {
        return;
      }
    }

    if (node_M.is_leaf()) {
      for (int i = 0; i < node_M.num_element_indices(); ++i) {
        const int triangle_index = node_M.element_index(i);
        const SquaredDistanceToTriangle squared_distance =
            CalcSquaredDistanceToTriangle(p_MQ_, triangle_index, mesh_M_,
                                          normal_set_M_);
        if (squared_distance.squared_distance <
            closest_info_.squared_distance) {
          closest_info_ = squared_distance;
        }
      }
      return;
    }

    // We couldn't prune the subtree, recursively search both children.
    this->Visit(node_M.left());
    this->Visit(node_M.right());
  }

 private:
  const Vector3<double>& p_MQ_;
  const TriangleSurfaceMesh<double>& mesh_M_;
  const FeatureNormalSet& normal_set_M_;

  SquaredDistanceToTriangle closest_info_;
};

}  // namespace

std::variant<FeatureNormalSet, std::string> FeatureNormalSet::MaybeCreate(
    const TriangleSurfaceMesh<double>& mesh_M) {
  std::vector<Vector3<double>> vertex_normals(mesh_M.num_vertices(),
                                              Vector3d::Zero());
  std::unordered_map<SortedPair<int>, Vector3<double>> edge_normals;
  const std::vector<Vector3d>& vertices = mesh_M.vertices();
  // We can compute a tolerance for a knife edge based on a minimum dihedral
  // angle θ between adjacent faces using the following formula:
  // ‖nᵢ + nⱼ‖² = 2(1-cos(θ)), where nᵢ and nⱼ are the face normals of the
  // two adjacent faces.
  //     The case for "pointy" vertices is more complex and requires more
  // assumptions and guesswork. Empirically, the pointy vertices have the
  // same approximate magnitude as for the edge tolerance, but we won't get
  // into deriving the tolerance for simplicity's sake.
  constexpr double kToleranceSquaredNorm = 3e-6;

  // Accumulate data from the mesh. They are not normal vectors yet. We will
  // normalize them afterward.
  for (int f = 0; f < mesh_M.num_triangles(); ++f) {
    const Vector3d& face_normal = mesh_M.face_normal(f);
    const SurfaceTriangle& tri = mesh_M.triangles()[f];
    const int v[3] = {tri.vertex(0), tri.vertex(1), tri.vertex(2)};
    const Vector3d unit_edge_vector[3] = {
        (vertices[v[1]] - vertices[v[0]]).normalized(),
        (vertices[v[2]] - vertices[v[1]]).normalized(),
        (vertices[v[0]] - vertices[v[2]]).normalized()};

    // Accumulate angle*normal for each vertex of the triangle.
    for (int i = 0; i < 3; ++i) {
      const double angle =
          std::acos(unit_edge_vector[i].dot(-unit_edge_vector[(i + 2) % 3]));
      vertex_normals[v[i]] += angle * face_normal;
    }
    // Accumulate normal for each edge of the triangle.
    for (int i = 0; i < 3; ++i) {
      const auto edge = MakeSortedPair(v[i], v[(i + 1) % 3]);
      auto it = edge_normals.find(edge);
      if (it == edge_normals.end()) {
        edge_normals[edge] = face_normal;
      } else {
        it->second += face_normal;
        if (it->second.squaredNorm() < kToleranceSquaredNorm) {
          return "FeatureNormalSet: Cannot compute an edge normal because "
                 "the two triangles sharing the edge make a very sharp edge.";
        }
        it->second.normalize();
      }
    }
  }
  for (auto& v_normal : vertex_normals) {
    if (v_normal.squaredNorm() < kToleranceSquaredNorm) {
      return "FeatureNormalSet: Cannot compute a vertex normal because "
             "the triangles sharing the vertex form a very pointy needle.";
    }
    v_normal.normalize();
  }

  return FeatureNormalSet(std::move(vertex_normals), std::move(edge_normals));
}

SquaredDistanceToTriangle CalcSquaredDistanceToTriangle(
    const Vector3<double>& p_MQ, int triangle_index,
    const TriangleSurfaceMesh<double>& mesh_M,
    const FeatureNormalSet& normal_set_M) {
  // Barycentric coordinates of the projection of Q on the plane of the
  // triangle.
  Vector3d b_Q = mesh_M.CalcBarycentric(p_MQ, triangle_index);

  // We've defined nearest feature with priority: vertex, edge, face. However,
  // it's simple to determine if the answer is face -- all barycentric
  // coordinates are strictly positive. If that is not the case, we'll resolve
  // vertex vs. edge below.
  if (b_Q(0) > 0 && b_Q(1) > 0 && b_Q(2) > 0) {
    // The projection of Q onto the plane of the triangle is in the triangle,
    // so the nearest point is at the projection.
    const Vector3d p_MN =
        mesh_M.CalcCartesianFromBarycentric(triangle_index, b_Q);
    return {(p_MQ - p_MN).squaredNorm(), p_MN,
            mesh_M.face_normal(triangle_index)};
  }

  const SurfaceTriangle& triangle = mesh_M.triangles()[triangle_index];
  const std::array<Vector3d, 3> p_MV = {mesh_M.vertex(triangle.vertex(0)),
                                        mesh_M.vertex(triangle.vertex(1)),
                                        mesh_M.vertex(triangle.vertex(2))};

  // The closest point is either in an edge or at a vertex. We will search
  // the three edges.
  SquaredDistanceToTriangle result;
  // Iterate over three edges, call each edge AB.
  int prev_i = 2;
  for (int i = 0; i < 3; ++i) {
    const Vector3d p_MA = p_MV[prev_i];
    const Vector3d p_MB = p_MV[i];
    const Vector3d p_AB_M = p_MB - p_MA;
    // t = 0 when the projection is at A, and t = 1 at B.
    const double t = p_AB_M.dot(p_MQ - p_MA) / p_AB_M.squaredNorm();
    // N is the nearest point in the line segment.
    Vector3d p_MN;
    Vector3d normal_M;
    if (t <= 0) {
      p_MN = p_MA;
      normal_M = normal_set_M.vertex_normal(triangle.vertex(prev_i));
    } else if (t >= 1) {
      p_MN = p_MB;
      normal_M = normal_set_M.vertex_normal(triangle.vertex(i));
    } else {
      p_MN = (1.0 - t) * p_MA + t * p_MB;
      normal_M = normal_set_M.edge_normal(
          {triangle.vertex(prev_i), triangle.vertex(i)});
    }

    const double d_squared = (p_MQ - p_MN).squaredNorm();
    if (d_squared < result.squared_distance) {
      result = SquaredDistanceToTriangle({d_squared, p_MN, normal_M});
    }
    prev_i = i;
  }
  return result;
}

SignedDistanceToSurfaceMesh CalcSignedDistanceToSurfaceMesh(
    const Vector3<double>& p_MQ, const TriangleSurfaceMesh<double>& mesh_M,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_M,
    const FeatureNormalSet& feature_normals_M) {
  SquaredDistanceToTriangle closest =
      BvhVisitor::CalcSquaredDistance(p_MQ, mesh_M, bvh_M, feature_normals_M);
  // N is the nearest point.
  const Vector3d p_MN = closest.closest_point;
  const Vector3d p_NQ_M = p_MQ - p_MN;
  const double sign = p_NQ_M.dot(closest.feature_normal) >= 0 ? 1 : -1;
  const double unsigned_distance = std::sqrt(closest.squared_distance);
  return {.signed_distance = sign * unsigned_distance,
          .nearest_point = p_MN,
          .gradient = unsigned_distance == 0 ? closest.feature_normal
                                             : sign * p_NQ_M.normalized()};
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
