#include "drake/geometry/proximity/bounding_volume_hierarchy.h"

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using math::RotationMatrixd;

bool Aabb::HasOverlap(const Aabb& a, const Aabb& b,
                      const math::RigidTransformd& X_AB) {
  // We need to split the transform into the position and rotation components,
  // `p_AB` and `R_AB`. For the purposes of streamlining the math below, they
  // will henceforth be named `t` and `r` respectively.
  const Vector3d t = X_AB * b.center() - a.center();
  const Matrix3d r = X_AB.rotation().matrix();

  // Compute some common subexpressions and add epsilon to counteract
  // arithmetic error, e.g. when two edges are parallel. We use the value as
  // specified from Gottschalk's OBB robustness tests.
  const double kEpsilon = 0.000001;
  Matrix3d abs_r = r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      abs_r(i, j) = abs(abs_r(i, j)) + kEpsilon;
    }
  }

  // First category of cases separating along a's axes.
  for (int i = 0; i < 3; ++i) {
    if (abs(t[i]) >
        a.half_width()[i] + b.half_width().dot(abs_r.block<1, 3>(i, 0))) {
      return false;
    }
  }

  // Second category of cases separating along b's axes.
  for (int i = 0; i < 3; ++i) {
    if (abs(t.dot(r.block<3, 1>(0, i))) >
        b.half_width()[i] + a.half_width().dot(abs_r.block<3, 1>(0, i))) {
      return false;
    }
  }

  // Third category of cases separating along the axes formed from the cross
  // products of a's and b's axes.
  int i1 = 1;
  for (int i = 0; i < 3; ++i) {
    const int i2 = (i1 + 1) % 3;  // Calculate common sub expressions.
    int j1 = 1;
    for (int j = 0; j < 3; ++j) {
      const int j2 = (j1 + 1) % 3;
      if (abs(t[i2] * r(i1, j) -
              t[i1] * r(i2, j)) >
          a.half_width()[i1] * abs_r(i2, j) +
              a.half_width()[i2] * abs_r(i1, j) +
              b.half_width()[j1] * abs_r(i, j2) +
              b.half_width()[j2] * abs_r(i, j1)) {
        return false;
      }
      j1 = j2;
    }
    i1 = i2;
  }

  return true;
}

bool Aabb::HasOverlap(const Aabb& bv, const Plane<double>& plane_P,
                      const math::RigidTransformd& X_PH) {
  // We want the two corners of the box that lie at the most extreme extents in
  // the plane's normal direction. Then we can determine their heights
  // -- if the interval of heights includes _zero_, the box overlaps.

  // The box's canonical frame B is aligned with H; R_BH = I which implies
  // R_PH = R_PB. However, p_HoBo is not necessarily zero.
  const auto& R_PH = X_PH.rotation().matrix();
  // The corner of the box that will have the *greatest* height value w.r.t.
  // the plane measured from the box's frame's origin (Bo) but expressed in the
  // plane's frame.
  Vector3d p_BoCmax_P = Vector3d::Zero();
  // We want to compute the vectors Hᴹᵢ  ∈ {Hᵢ, -Hᵢ}, such that Hᴹᵢ ⋅ n̂ₚ is
  // positive. The maximum box corner is a combination of those Hᴹᵢ vectors.
  for (int i = 0; i < 3; ++i) {
    const Vector3d& Hi_P = R_PH.col(i);
    const Vector3d& Hi_max_P = Hi_P.dot(plane_P.normal()) > 0 ? Hi_P : -Hi_P;
    p_BoCmax_P += Hi_max_P * bv.half_width()(i);
  }

  const Vector3d& p_HoBo_H = bv.center();
  const Vector3d p_PoBo_P = X_PH * p_HoBo_H;
  // Minimum corner is merely the reflection of the maximum corner across the
  // center of the box.
  const Vector3d p_PoCmax_P = p_PoBo_P + p_BoCmax_P;
  const Vector3d p_PoCmin_P = p_PoBo_P - p_BoCmax_P;

  const double max_height = plane_P.CalcHeight(p_PoCmax_P);
  const double min_height = plane_P.CalcHeight(p_PoCmin_P);
  return min_height <= 0 && 0 <= max_height;
}

bool Aabb::HasOverlap(const Aabb& bv, const HalfSpace&,
                      const math::RigidTransformd& X_CH) {
  /*
                                              Hy           Hx
                                                ╲        ╱
                        By  ╱╲       Bx          ╲      ╱
                          ╲╱  ╲    ╱              ╲    ╱
                          ╱╲   ╲  ╱                ╲  ╱
                         ╱  ╲   ╲╱                  ╲╱
                         ╲   ╲  ╱╲   bv               Ho
                          ╲   ╲╱  ╲
                           ╲   Bo  ╲
                            ╲       ╲
                             ╲      ╱
                              ╲    ╱
                               ╲  ╱        Cz
                                ╲╱         ^
                                L          ┃  Cx
              ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┺━━━>┄┄┄┄┄┄┄┄┄┄┄┄┄
              ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  Half space
              ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░

    If any point in the bounding volume has a signed distance φ that is less
    than or equal to zero, we consider the box to be overlapping the half space.
    We could simply, yet inefficiently, determine this by iterating over all
    eight vertices and evaluating the signed distance for each vertex.

    However, to provide value as a culling algorithm, we need to be cheaper. So,
    if the lowest corner (marked `L`) has a signed distance less than or equal
    to zero, the overlapping condition is met.

    The point L = Bₒ + ∑ sᵢ * dᵢ * Bᵢ, where:
      - i ∈ {x, y, z}.
      - dᵢ is the _half_ measure of the box's dimension along axis i.
      - sᵢ ∈ {1, -1}, such that sᵢBᵢ ⋅ Cz ≤ 0.

    Since, φ(p_CL) = p_CL ⋅ Cz. If p_CL is expressed in C, then the z-component
    of p_CL (p_CL_z), is equal to φ(p_CL). So, if p_CL_z ≤ 0, they overlap.
   */

  // The z-component of the position vector from box center (Bo) to the lowest
  // corner of the box (L) expressed in the half space's canonical frame C.
  const auto& R_CH = X_CH.rotation().matrix();
  double p_BL_C_z = 0.0;
  for (int i = 0; i < 3; ++i) {
    // R_CH(2, i) is Hi_C(2) --> the z-component of Hi_C.
    const double Hi_C_z = R_CH(2, i);
    const double s_i = Hi_C_z > 0 ? -1 : 1;
    p_BL_C_z += s_i * bv.half_width()(i) * Hi_C_z;
  }
  // Now we compute the z-component of the position vector from Co to L,
  // expressed in Frame C.
  //  p_CL_C = p_CB_C                   + p_BL_C
  //         = p_CH_C + p_HB_C          + p_BL_C
  //         = p_CH_C + (R_CH * p_HB_H) + p_BL_C
  // In all of these calculations, we only need the z-component. So, that means
  // we can get the z-component of p_HB_C without the full
  // R_CH * p_HB_H calculation; we can simply do Cz_H ⋅ p_HB_H.
  const Vector3d& p_HB_H = bv.center();
  const Vector3d& Cz_H = R_CH.row(2);
  const double p_HB_C_z = Cz_H.dot(p_HB_H);
  const double p_CH_C_z = X_CH.translation()(2);
  const double p_CB_C_z = p_CH_C_z + p_HB_C_z;
  const double p_CL_C_z = p_CB_C_z + p_BL_C_z;
  return p_CL_C_z <= 0;
}

void Aabb::PadBoundary() {
  const double max_position = center_.cwiseAbs().maxCoeff();
  const double max_half_width = half_width_.maxCoeff();
  const double scale = std::max(max_position, max_half_width);
  const double incr =
      std::max(scale * std::numeric_limits<double>::epsilon(), kTolerance);
  half_width_ += Vector3d::Constant(incr);
}

template <class MeshType>
BoundingVolumeHierarchy<MeshType>::BoundingVolumeHierarchy(
    const MeshType& mesh) {
  // Generate element indices and corresponding centroids. These are used
  // for calculating the split point of the volumes.
  const int num_elements = mesh.num_elements();
  std::vector<CentroidPair> element_centroids;
  for (IndexType i(0); i < num_elements; ++i) {
    element_centroids.emplace_back(i, ComputeCentroid(mesh, i));
  }

  root_node_ =
      BuildBVTree(mesh, element_centroids.begin(), element_centroids.end());
}

template <class MeshType>
std::unique_ptr<BvNode<MeshType>>
BoundingVolumeHierarchy<MeshType>::BuildBVTree(
    const MeshType& mesh,
    const typename std::vector<CentroidPair>::iterator& start,
    const typename std::vector<CentroidPair>::iterator& end) {
  // Generate bounding volume.
  Aabb aabb = ComputeBoundingVolume(mesh, start, end);

  const int num_elements = end - start;
  if (num_elements == 1) {
    // Store element index in this leaf node.
    return std::make_unique<BvNode<MeshType>>(aabb, start->first);

  } else {
    // Sort the elements by centroid along the axis of greatest spread.
    // Note: We tried an alternative strategy for building the BVH using a
    // volume-based metric.
    // - Given a parent BV P, we would partition all of its contents into a left
    //   BV, L, and a right BV, R, such that we wanted to minimize (V(L) + V(R))
    //   / V(P). (V(X) is the volume measure of the bounding volume X).
    // - We didn't explore all possible partitions (there are an exponential
    //   number of such possible partitions).
    // - Instead, we ordered the mesh elements along an axis and then would
    //   consider partitions split between two adjacent elements in the sorted
    //   set. This was repeated for each of the axes and the partition with the
    //   minimum value was taken.
    // - We did explore several ordering options, including sorting by centroid,
    //   min, max, and a combination of them when the element overlapped the
    //   partition boundary.
    // This tentative partitioning strategy produced more BV-BV tests in some
    // simple examples than the simple bisection shown below and was abandoned.
    // Some possible reasons for this:
    // - Sorting by an alternate criteria might be a better way to order them.
    // - Only considering split points based on adjacent elements may be
    //   problematic.
    // - The elements individual extents are such they are not typically
    //   axis-aligned so, partitioning between two elements would often produce
    //   child BVs that have non-trivial overlap.
    // Finally, the primitive meshes we are producing are relatively regular and
    // are probably nicely compatible with the median split strategy. If we need
    // to do irregular distribution of elements, a more sophisticated strategy
    // may help. But proceed with caution -- there's no guarantee such a
    // strategy will yield performance benefits.
    int axis{};
    aabb.half_width().maxCoeff(&axis);
    std::sort(start, end, [axis](const CentroidPair& a, const CentroidPair& b) {
      return a.second[axis] < b.second[axis];
    });

    // Continue with the next branches.
    const typename std::vector<CentroidPair>::iterator mid =
        start + num_elements / 2;
    return std::make_unique<BvNode<MeshType>>(
        aabb, BuildBVTree(mesh, start, mid), BuildBVTree(mesh, mid, end));
  }
}

template <class MeshType>
Aabb BoundingVolumeHierarchy<MeshType>::ComputeBoundingVolume(
    const MeshType& mesh,
    const typename std::vector<CentroidPair>::iterator& start,
    const typename std::vector<CentroidPair>::iterator& end) {
  // Keep track of the min/max bounds to create the bounding box.
  Vector3d max_bounds, min_bounds;
  max_bounds.setConstant(std::numeric_limits<double>::lowest());
  min_bounds.setConstant(std::numeric_limits<double>::max());

  // Check each mesh element in the given range.
  for (auto pair = start; pair < end; ++pair) {
    const auto& element = mesh.element(pair->first);
    // Check each vertex in the element.
    for (int v = 0; v < kElementVertexCount; ++v) {
      const auto& vertex = mesh.vertex(element.vertex(v)).r_MV();
      // Compare its extent along each of the 3 axes.
      min_bounds = min_bounds.cwiseMin(vertex);
      max_bounds = max_bounds.cwiseMax(vertex);
    }
  }
  const Vector3d center = (min_bounds + max_bounds) / 2;
  const Vector3d half_width = max_bounds - center;
  return Aabb(center, half_width);
}

template <class MeshType>
Vector3d BoundingVolumeHierarchy<MeshType>::ComputeCentroid(
    const MeshType& mesh, const IndexType i) {
  Vector3d centroid{0, 0, 0};
  const auto& element = mesh.element(i);
  // Calculate average from all vertices.
  for (int v = 0; v < kElementVertexCount; ++v) {
    const auto& vertex = mesh.vertex(element.vertex(v)).r_MV();
    centroid += vertex;
  }
  centroid /= kElementVertexCount;
  return centroid;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake

template class drake::geometry::internal::BoundingVolumeHierarchy<
    drake::geometry::SurfaceMesh<double>>;
template class drake::geometry::internal::BoundingVolumeHierarchy<
    drake::geometry::VolumeMesh<double>>;
