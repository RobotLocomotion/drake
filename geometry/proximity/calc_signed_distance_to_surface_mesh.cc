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

  SquaredDistanceToTriangle closest_info_{
      .squared_distance = std::numeric_limits<double>::infinity()};
};

}  // namespace

FeatureNormalSet::FeatureNormalSet(const TriangleSurfaceMesh<double>& mesh_M)
    : vertex_normals_(mesh_M.num_vertices(), Vector3d::Zero()) {
  const std::vector<Vector3d>& vertices = mesh_M.vertices();
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
      vertex_normals_[v[i]] += angle * face_normal;
    }
    // Accumulate normal for each edge of the triangle.
    for (int i = 0; i < 3; ++i) {
      const auto edge = MakeSortedPair(v[i], v[(i + 1) % 3]);
      auto it = edge_normals_.find(edge);
      if (it == edge_normals_.end()) {
        edge_normals_[edge] = face_normal;
      } else {
        it->second += face_normal;
        {
          // To guard against normalizing a near-zero vector of numerical
          // noises, which could happen when two triangles sharing
          // an edge make a near-zero dihedral angle, we will check the
          // squared norm of the sum of the two face normals against the
          // threshold derived from this very small dihedral angle in degrees.
          constexpr double kThetaDegree = 1;
          // Two triangles making a small dihedral angle θ has their
          // face-normal sum with the squared norm 2(1-cos(θ)). For
          // example, one normal is (1, 0, 0) and another one is
          // (-cos(θ), -sin(θ), 0). Their sum is (1-cos(θ), -sin(θ), 0) with
          // the squared norm (1-cos(θ))² + sin(θ)² = 2(1-cos(θ)).
          // For θ = 1 degree, this threshold is about 3e-4.
          static const double kThresholdSquaredNorm =
              2 * (1 - std::cos(kThetaDegree * M_PI / 180));
          if (it->second.squaredNorm() < kThresholdSquaredNorm) {
            throw std::runtime_error(
                "FeatureNormalSet: Cannot compute an edge normal possibly "
                "because the two triangles sharing the edge make a dihedral "
                "angle less than one degree, or the two triangles have "
                "opposite face windings.");
          }
        }
        it->second.normalize();
      }
    }
  }
  for (auto& v_normal : vertex_normals_) {
    // It is possible that a vertex has a near-zero angle-weighted sum of
    // face normals with all incident edges passing the above guard.
    // To guard against such vertices, we will check the squared norm of
    // the angle-weighted sum against the threshold derived from this very
    // small face angle in degrees.
    constexpr double kFaceAngleDegree = 1;
    constexpr double kFaceAngleRadian = kFaceAngleDegree * M_PI / 180;
    // At the apex vertex shared by four congruent isosceles triangles
    // forming a square pyramid, the squared norm of the angle-weighted
    // sum of their face normals is 16 (α tan(α/2))², where α is the
    // face angle at the apex.
    //     For example, a square pyramid has its apex at (0, 0, h) and the
    // other four vertices at (±r, ±r, 0). Its height is h, its side length
    // is 2r, and its slant height (height of the isosceles triangle) is
    // s = √(r² + h²). Each triangle has slope h/r, so each face normal
    // has slope r/h. Therefore, each face normal nᵢ has its Z component
    // equals r/√(r² + h²) = r/s. It follows that the sum of the face
    // normal is:
    //                    ∑nᵢ = (0, 0, 4r/s)
    // because by symmetry, the X-Y components cancel out.
    // The angle-weighted sum of the face normals is:
    //                   ∑αnᵢ = (0, 0, 4αr/s).
    // The line from the apex to the midpoint of the base of the isosceles
    // triangle cut it into two congruent right triangles with
    // angle α/2 at the apex. In this right triangle, the angle α/2 is
    // opposite to the base r and next to the height s, so we have:
    //               tan(α/2) = r/s.
    // Substituting r/s by tan(α/2) in the angle-weighted sum:
    //                   ∑αnᵢ = (0, 0, 4α tan(α/2)),
    // and its squared norm is:
    //                ‖∑αnᵢ‖² = (4α tan(α/2))².
    // This quantity is independent of the frame and the scale of the pyramid.
    //
    // For the one-degree face angle threshold, this squared norm threshold
    // is about 3e-7.
    static const double kAngleWeightedSumSquaredNorm =
        std::pow(4 * kFaceAngleRadian * std::tan(kFaceAngleRadian / 2), 2);
    if (v_normal.squaredNorm() < kAngleWeightedSumSquaredNorm) {
      throw std::runtime_error(
          "FeatureNormalSet: Cannot compute a vertex normal possibly "
          "because the triangles sharing the vertex form a very pointy "
          "feature. For example, an apex of a right square pyramid with "
          "each of the four face angles less than one degree.");
    }
    v_normal.normalize();
  }
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
