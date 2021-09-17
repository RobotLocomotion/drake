#include "drake/geometry/proximity/contact_surface_utility.h"

#include <algorithm>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/default_scalars.h"

namespace drake {

namespace geometry {
namespace internal {
namespace {

/* Utility for CalcPolygonCentroid() to evaluate the correctness of the inputs
 to CalcPolygonCentroid(). CalcPolygonCentroid() uses area computations to
 compute the centroid position and this method verifies that the parameter
 values are consistent with that effort. It throws only if the parameters
 would prevent that function from computing meaningful areas. The following
 conditions are cases in which that would occur.

   1. If the normal is "too short" -- approximately zero. Technically, non-zero
      vectors would be sufficient, but we want to make sure the caller is
      passing vectors they have confidence in and not rounding error-derived
      vectors.
   2. The polygon vertices are not planar.
   3. The normal is "insufficiently" perpendicular to the polygon plane. If the
      normal is parallel with the polygon plane, area will be _incorrectly_
      calculated to be zero. Perpendicularity prevents that.
      CalcPolygonCentroid() is documented as requiring a truly perpendicular.
      normal In practice it is not necessary; we just need the normal to clearly
      not lie on the polygon's plane. So, to be friendly we allow a fair amount
      of tolerance under the hood, knowing we can always tighten it later.

  NOTE: If the polygon is shown to be degenerate (having zero area), we don't
  have to care about #3 above. This is because, even if the normal is parallel,
  the resultant zero area is correct. So, if the end result is unaffected, it's
  not worth throwing.
  */
template <typename T>
void ThrowIfInvalidForCentroid(const char* prefix,
                               const std::vector<int>& polygon,
                               const Vector3<T>& n_F,
                               const std::vector<Vector3<T>>& vertices_F) {
  // TODO(SeanCurtis-TRI): Consider also validating convexity.

  // First test for sufficient length.
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
    const Vector3<T>& v = vertices_F[polygon[i]];
    A.block(i, 0, 1, 4) << v(0), v(1), v(2), 1;
  }

  Eigen::FullPivLU<MatrixX<T>> lu(A);

  if (lu.dimensionOfKernel() == 0) {
    // A kernel with dimension equal to zero implies the null space consists of
    // a single point.
    // https://eigen.tuxfamily.org/dox/classEigen_1_1FullPivLU.html
    throw std::runtime_error(
        fmt::format("{}: input polygon is not planar", prefix));
  }

  if (lu.dimensionOfKernel() > 1) {
    // This is a degenerate (zero-area) polygon. The normal really won't matter.
    return;
  }

  // The kernel is the null space of A. We know it has a single vector and we
  // take the first three entries as the normal.
  const Vector3<T> plane_norm = lu.kernel().block(0, 0, 3, 1).normalized();

  // We've stated "@pre n_F is perpendicular to polygon". This is sleight of
  // hand so the caller makes an effort. In practice,  as long as we're well
  // away from co-planar, the answer is fine. So, we'll test it against a
  // large cone (with an approximately 90-degree angle). We  can always tighten
  // it later, if necessary.

  // NOTE: We have no guarantee that n_F and plane_norm are pointing to the same
  // side of the polygon plane, simply throw out the sign of the dot product.
  using std::abs;
  if (abs(plane_norm.dot(n_F.normalized())) < 0.7071) {
    throw std::runtime_error(
        fmt::format("{}: the given normal is not perpendicular to the "
                    "polygon's plane; given normal: [{}], plane normal: [{}]",
                    prefix, n_F.transpose(), plane_norm.transpose()));
  }
}

}  // namespace

template <typename T>
Vector3<T> CalcPolygonCentroid(const std::vector<int>& polygon,
                               const Vector3<T>& n_F,
                               const std::vector<Vector3<T>>& vertices_F) {
  // The position of the geometric centroid can be computed by decomposing the
  // polygon into triangles and performing an area-weighted average of each of
  // the triangle's centroids.
  // See https://en.wikipedia.org/wiki/Centroid#By_geometric_decomposition.
  const int v_count = static_cast<int>(polygon.size());
  DRAKE_DEMAND(v_count >= 3);
  DRAKE_ASSERT_VOID(ThrowIfInvalidForCentroid("CalcPolygonCentroid", polygon,
                                              n_F, vertices_F));

  auto triangle_centroid = [&vertices_F](int v0, int v1, int v2) {
    return (vertices_F[v0] + vertices_F[v1] + vertices_F[v2]) / 3;
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
  // The polygon centroid is a weighted average of each triangle's centroid:
  // ∑(Aᵢ * centroidᵢ) / ∑Aᵢ, where Aᵢ and centroidᵢ
  // are the area and centroid of the ith triangle in the fan.

  // We're not using actual triangle area as our weight. We're using some
  // arbitrary scale of area, k. However,
  // ∑(Aᵢ * centroidᵢ) / ∑Aᵢ = ∑(kAᵢ * centroidᵢ) / ∑kAᵢ, k != 0.
  // The value of k comes from the scale and orientation of n_F.
  auto triangle_weight = [&vertices_F, &n_F](int v0, int v1, int v2) {
    const Vector3<T>& r_MV0 = vertices_F[v0];
    const Vector3<T>& r_MV1 = vertices_F[v1];
    const Vector3<T>& r_MV2 = vertices_F[v2];
    return (r_MV1 - r_MV0).cross(r_MV2 - r_MV0).dot(n_F);
  };

  Vector3<T> p_FC_accum = Vector3<T>::Zero();
  T total_weight{0};

  // Create the triangle fan about v0 described above.
  const int v0 = polygon[0];
  int v2 = polygon[1];
  for (int i = 2; i < v_count; ++i) {
    const int v1 = v2;
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
      p_FC_accum += vertices_F[polygon[i]];
    }
    total_weight = v_count;
  }

  return p_FC_accum / total_weight;
}

template <typename T>
Vector3<T> CalcPolygonCentroid(const std::vector<Vector3<T>>& p_FVs,
                               const Vector3<T>& n_F) {
  int num_vertices = p_FVs.size();
  std::vector<int> polygon(num_vertices);
  std::iota(polygon.begin(), polygon.end(), 0);

  return CalcPolygonCentroid(polygon, n_F, p_FVs);
}

template <typename T>
T CalcPolygonArea(const std::vector<Vector3<T>>& p_FVs,
                  const Vector3<T>& nhat_F) {
  int num_vertices = p_FVs.size();
  DRAKE_DEMAND(num_vertices >= 3);

  T total_double_area{0};
  const Vector3<T>& p_FA = p_FVs[0];
  for (int i = 2; i < num_vertices; ++i) {
    // Adding area of sub-triangles of the given polygon. For example, the
    // polygon V₀V₁V₂V₃V₄ will have these sub-triangles:
    //  i    triangle ABC
    //  2       V₀V₁V₂
    //  3       V₀V₂V₃
    //  4       V₀V₃V₄
    const Vector3<T> p_FB = p_FVs[i-1];
    const Vector3<T> p_FC = p_FVs[i];
    const Vector3<T> ABxAC_F = (p_FB - p_FA).cross(p_FC - p_FA);
    T double_area = ABxAC_F.dot(nhat_F);
    total_double_area += double_area;
  }
  return total_double_area / 2;
}

template <typename T>
void AddPolygonToMeshData(const std::vector<int>& polygon,
                          const Vector3<T>& n_F,
                          std::vector<SurfaceFace>* faces,
                          std::vector<Vector3<T>>* vertices_F) {
  DRAKE_DEMAND(faces != nullptr);
  DRAKE_DEMAND(vertices_F != nullptr);
  DRAKE_DEMAND(polygon.size() >= 3);

  // The polygon will be represented by an equivalent triangle fan around the
  // polygon's centroid. This requires add a new vertex: the centroid.
  Vector3<T> p_FC = CalcPolygonCentroid(polygon, n_F, *vertices_F);
  const int centroid_index = static_cast<int>(vertices_F->size());
  vertices_F->emplace_back(p_FC);

  // The first thing we do in the for loop is v1 = v2, so this guarantees that
  // v1 will be polygon[N-1] in the first iteration. We'll get triangles:
  //  (N-1, 0, centroid)
  //  (0, 1, centroid)
  //  (1, 2, centroid)
  //  ...
  int v2{polygon.back()};
  const int polygon_size = static_cast<int>(polygon.size());
  for (int i = 0; i < polygon_size; ++i) {
    const int v1 = v2;
    v2 = polygon[i];
    faces->emplace_back(v1, v2, centroid_index);
  }
}

template <typename T>
void AddPolygonToMeshDataAsOneTriangle(const std::vector<Vector3<T>>& polygon_F,
                                       const Vector3<T>& nhat_F,
                                       std::vector<SurfaceFace>* faces,
                                       std::vector<Vector3<T>>* vertices_F) {
  DRAKE_DEMAND(faces != nullptr);
  DRAKE_DEMAND(vertices_F != nullptr);
  DRAKE_DEMAND(polygon_F.size() >= 3);
  const T polygon_area = CalcPolygonArea(polygon_F, nhat_F);
  if (polygon_area < kMinimumPolygonArea) {
    return;
  }

  // Locations of vertices of the representative triangle.
  Vector3<T> p_FVs[3];
  if (polygon_F.size() == 3) {
    p_FVs[0] = polygon_F[0];
    p_FVs[1] = polygon_F[1];
    p_FVs[2] = polygon_F[2];
  } else {
    Vector3<T> p_FC = CalcPolygonCentroid(polygon_F, nhat_F);

    // We will set the representative triangle as an isosceles right triangle
    // in a local frame G such that its third basis vector Gz_F, expressed in
    // frame F, is the same as the face normal vector nhat_F. The triangle
    // has centroid C (coincident with the polygon's centroid) at Go, and its
    // two edges V₀V₁ and V₀V₂ are parallel to Gx and Gy respectively, as shown
    // in the following picture (Gz points outward from the screen).
    //
    //               Gy
    //               ^
    //               |
    //        V₂ ●   |
    //           x x |
    //           x   x
    //        l  x   | x
    //           x C ○---x--------> Gx
    //           x         x
    //       V₀  ● x x x x x ●
    //                 l     V₁
    //
    // The two edges of the triangle in Gx and Gy directions have the same
    // length l such that the triangle has the same area as the polygon:
    //      l²/2 = polygon_area
    //      l    = √(2 * polygon_area)
    // N.B. We should have positive polygon_area, so the scalar type AutoDiffXd
    // would be fine with sqrt().
    const T l = sqrt(2. * polygon_area);
    // Pass axis_index = 2, so that Gz_F = R_FG.col(2) = nhat_F.
    const auto R_FG = math::RotationMatrix<T>::MakeFromOneUnitVector(nhat_F, 2);
    const Vector3<T> Gx_F = R_FG.col(0);
    const Vector3<T> Gy_F = R_FG.col(1);
    p_FVs[0] = p_FC + (l / 3) * (   -Gx_F      - Gy_F);  // V₀ #NOLINT
    p_FVs[1] = p_FC + (l / 3) * (2 * Gx_F      - Gy_F);  // V₁ #NOLINT
    p_FVs[2] = p_FC + (l / 3) * (   -Gx_F + 2. * Gy_F);  // V₂ #NOLINT
  }

  const int n = vertices_F->size();
  const int v[3] = {n, n + 1, n + 2};
  faces->emplace_back(v);
  vertices_F->emplace_back(p_FVs[0]);
  vertices_F->emplace_back(p_FVs[1]);
  vertices_F->emplace_back(p_FVs[2]);
}

template <typename T>
bool IsFaceNormalInNormalDirection(const Vector3<T>& normal_F,
                                   const SurfaceMesh<T>& surface_M,
                                   int tri_index,
                                   const math::RotationMatrix<T>& R_FM) {
  const Vector3<T>& face_normal_F = R_FM * surface_M.face_normal(tri_index);
  // Given the rotation, we're re-normalizing the face normal to guard against
  // round off error introduced by the rotation; we don't know its history and
  // how much error it has accumulated.
  // Theta is the angle between normal_F and face_normal_F. The dot product
  // produces the cos(theta) if both normals are unit length).
  const T cos_theta = normal_F.dot(face_normal_F.normalized());

  // We pick 5π/8 empirically to be the threshold angle, alpha.
  constexpr double kAlpha = 5. * M_PI / 8.;
  static const double kCosAlpha = std::cos(kAlpha);
  // cos(θ) > cos(α) → θ < α → condition met.
  return cos_theta > kCosAlpha;
}

// Instantiation to facilitate unit testing of this support function.
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    &AddPolygonToMeshData<T>,
    &AddPolygonToMeshDataAsOneTriangle<T>,
    &CalcPolygonArea<T>,
    &IsFaceNormalInNormalDirection<T>,
    /* Use static_cast to disambiguate the two different overloads. */
    static_cast<Vector3<T>(*)(
       const std::vector<int>&,
       const Vector3<T>&,
       const std::vector<Vector3<T>>&)>(&CalcPolygonCentroid),
    /* Use static_cast to disambiguate the two different overloads. */
    static_cast<Vector3<T>(*)(
       const std::vector<Vector3<T>>&,
       const Vector3<T>&)>(&CalcPolygonCentroid)
))

}  // namespace internal
}  // namespace geometry
}  // namespace drake
