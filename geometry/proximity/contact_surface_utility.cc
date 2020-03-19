#include "drake/geometry/proximity/contact_surface_utility.h"

#include <fmt/format.h>
#include <fmt/ostream.h>

namespace drake {

namespace geometry {
namespace internal {

/* Utility for CalcPolygonCentroid() to evaluate the correctness of the input
 normal vector. Determines if the given `nhat_F` is sufficiently unit length
 and sufficiently peprpendicular to the polygon's plane.  */
template <typename T>
void ValidatePolygon(const char* prefix,
                     const std::vector<SurfaceVertexIndex>& polygon,
                     const Vector3<T>& n_F,
                     const std::vector<SurfaceVertex<T>>& vertices_F) {
  // TODO(SeanCurtis-TRI): Consider also validating convexity.

  // First test for unit length.
  using std::abs;
  if (n_F.norm() < 1e-10) {
    throw std::runtime_error(fmt::format(
        "{}: given normal is too small; normal [{}] with length {}",
        prefix, n_F.transpose(), n_F.norm()));
  }

  // Now test for orthogonality.
  // We have no assurance as to the degeneracy of the input polygon. Inferring
  // the true polygon normal can be algorithmically tricky -- which three
  // vertices span the plane (if any)?
  // There's a plane equation (n⃗ ⋅ x⃗ + d = 0) that must be true for every point
  // in the polygon. Expressed in linear algebra it looks like this:
  //
  //  │ x₁ y₁ z₁ 1 │ │ nx │   │ 0 │
  //  │ ...        │⋅│ ny │ = │ ┇ │
  //  │ xₙ yₙ zₙ 1 │ │ nz │   │ 0 │
  //                 │ d  │
  //
  //  This is simply Ax = 0
  //
  // So, the plane normal can be extracted from the null space of A.
  // We have the following possible outcomes:
  //
  //   - The null space is *only* the zero vector; the vertices aren't planar.
  //   - The null space has a basis with multiple vectors; the polygon is
  //     degenerate and doesn't define a plane. It has zero area and we don't
  //     care about the normal.
  //   - the null space has a single, non-zero basis vector; this is the plane
  //     equation and we can extract the normal from it.
  MatrixX<T> A;
  const int v_count = static_cast<int>(polygon.size());
  A.resize(v_count, 4);
  for (int i = 0; i < v_count; ++i) {
    const Vector3<T>& v = vertices_F[polygon[i]].r_MV();
    A.block(i, 0, 1, 4) << v(0), v(1), v(2), 1;
  }
  Eigen::FullPivLU<MatrixX<T>> lu(A);
  MatrixX<T> A_null_space = lu.kernel();
  if (A_null_space.cols() > 1) {
    // This is a degenerate (zero-area) polygon. The normal really won't matter.
    return;
  }
  Vector3<T> plane_norm = A_null_space.block(0, 0, 3, 1);
  // TODO(SeanCurtis-TRI): What does it mean if the null space is simply Zero?
  // Check to see if the null space is the zero vector.
  if (plane_norm.norm() < 1e-13) {
    throw std::runtime_error(
        fmt::format("{}: input polygon is not planar", prefix));
  };
  plane_norm.normalize();

  // We're not *really* testing for true orthogonality; just a sufficient amount
  // of orthogonality.

  // NOTE: n_F must be perpendicular to the polygon's plane, but we have no
  // guarantees that it points in the same direction as `plane_norm`; simply
  // throw out the sign.
  if (abs(plane_norm.dot(n_F.normalized())) < 1e-8) {
    throw std::runtime_error(
        fmt::format("{}: the given normal is not perpendicular to the "
                    "polygon's plane; given normal: [{}], plane normal: [{}]",
                    prefix, n_F.transpose(), plane_norm.transpose()));
  }
}

template <typename T>
Vector3<T> CalcPolygonCentroid(
    const std::vector<SurfaceVertexIndex>& polygon,
    const Vector3<T>& n_F,
    const std::vector<SurfaceVertex<T>>& vertices_F) {
  // The position of the geometric centroid can be computed by decomposing the
  // polygon into triangles and performing an area-weighted average of each of
  // the triangle's centroids.
  // See https://en.wikipedia.org/wiki/Centroid#By_geometric_decomposition.
  const int v_count = static_cast<int>(polygon.size());
  DRAKE_DEMAND(v_count >= 3);
  DRAKE_ASSERT_VOID(
      ValidatePolygon("CalcPolygonCentroid", polygon, n_F, vertices_F));

  using V = SurfaceVertexIndex;

  auto triangle_centroid = [&vertices_F](V v0, V v1, V v2) {
    return (vertices_F[v0].r_MV() + vertices_F[v1].r_MV() +
        vertices_F[v2].r_MV()) /
        3;
  };

  // Triangles get special treatment.
  if (v_count == 3) {
    return triangle_centroid(polygon[0], polygon[1], polygon[2]);
  }

  // N-gon's have to be handled by geometric decomposition.
  // We'll decompose it by creating a triangle fan around vertex 0. I.e.,
  //   triangle 0: v0, v1, v2
  //   triangle 1: v0, v2, v3
  //   etc.
  // The polygon centroid is: ∑(kAᵢ * centroidᵢ) / ∑kAᵢ, where Aᵢ and centroidᵢ
  // are the area and centroid of the ith triangle in the fan. We can use the
  // area up to any scale (k != 0) because that scale factor gets divided right
  // back out.

  // This computes an area-based weight. In the ideal form, the magnitude of
  // the cross product is twice the area of the triangle. The value returned is
  // a scaled area based on two factors:
  //   - We don't need the actual area; any non-zero scale of the area would
  //     work just fine.
  //   - the normal (n_F) is not necessarily unit length.
  //   However, for a planar polygon, each decomposed triangle should produce a
  //   parallel cross product and dotting each of those with an _arbitrary_
  //   normal direction simply further scales the area by another constant,
  //   leaving the relative weights the same.
  auto triangle_weight = [&vertices_F, &n_F](V v0, V v1, V v2) {
    const Vector3<T>& r_MV0 = vertices_F[v0].r_MV();
    const Vector3<T>& r_MV1 = vertices_F[v1].r_MV();
    const Vector3<T>& r_MV2 = vertices_F[v2].r_MV();
    return (r_MV1 - r_MV0).cross(r_MV2 - r_MV0).dot(n_F);
  };

  Vector3<T> p_FC_accum = Vector3<T>::Zero();
  T total_weight{0};

  const V v0 = polygon[0];
  V v2 = polygon[1];
  for (int i = 2; i < v_count; ++i) {
    const V v1 = v2;
    v2 = polygon[i];
    const T weight = triangle_weight(v0, v1, v2);
    p_FC_accum += weight * triangle_centroid(v0, v1, v2);
    total_weight += weight;
  }

  if (total_weight == 0) {
    // The polygon is degenerate with no area. In that case, we'll simply
    // define the centroid as the average vertex position. (Alternatively we
    // could just *pick* one of the vertices.
    p_FC_accum = Vector3<T>::Zero();
    for (int i = 0; i < v_count; ++i) {
      p_FC_accum += vertices_F[polygon[i]].r_MV();
    }
    total_weight = v_count;
  }

  return p_FC_accum / total_weight;
}

template <typename T>
void AddPolygonToMeshData(
    const std::vector<SurfaceVertexIndex>& polygon,
    const Vector3<T>& n_F,
    std::vector<SurfaceFace>* faces,
    std::vector<SurfaceVertex<T>>* vertices_F) {
  DRAKE_DEMAND(faces != nullptr);
  DRAKE_DEMAND(vertices_F != nullptr);
  DRAKE_DEMAND(polygon.size() >= 3);

  // We're going to create a triangle fan for the polygon. This requires adding
  // a new vertex at the polygon's geometric centroid.
  Vector3<T> p_FC = CalcPolygonCentroid(polygon, n_F, *vertices_F);
  const SurfaceVertexIndex centroid_index(vertices_F->size());
  vertices_F->emplace_back(p_FC);

  // The first thing we do in the for loop is v1 = v2, so this guarantees that
  // v1 will be polygon[N-1] in the first iteration. We'll get triangles:
  //  (N-1, 0, centroid)
  //  (0, 1, centroid)
  //  (1, 2, centroid)
  //  ...
  SurfaceVertexIndex v2{polygon.back()};
  const int polygon_size = static_cast<int>(polygon.size());
  for (int i = 0; i < polygon_size; ++i) {
    const SurfaceVertexIndex v1 = v2;
    v2 = polygon[i];
    faces->emplace_back(v1, v2, centroid_index);
  }
}

template
Vector3<double> CalcPolygonCentroid(
    const std::vector<SurfaceVertexIndex>& polygon,
    const Vector3<double>& n_F,
    const std::vector<SurfaceVertex<double>>& vertices_F);

template void AddPolygonToMeshData(
    const std::vector<SurfaceVertexIndex>& polygon,
    const Vector3<double>& n_F,
    std::vector<SurfaceFace>* faces,
    std::vector<SurfaceVertex<double>>* vertices_F);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
