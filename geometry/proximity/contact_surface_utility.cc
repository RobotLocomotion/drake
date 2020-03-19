#include "drake/geometry/proximity/contact_surface_utility.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
Vector3<T> CalcPolygonCentroid(
    const std::vector<SurfaceVertexIndex>& polygon,
    const Vector3<T>& nhat_F,
    const std::vector<SurfaceVertex<T>>& vertices_F) {
  // The position of the geometric centroid can be computed by decomposing the
  // polygon into triangles and performing an area-weighted average of each of
  // the triangle's centroids.
  // See https://en.wikipedia.org/wiki/Centroid#By_geometric_decomposition.
  const int v_count = static_cast<int>(polygon.size());
  DRAKE_DEMAND(v_count >= 3);

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

  // This computes an area-based weight. In the purest form, the magnitude of
  // the cross product is twice the area of the triangle. The value returned is
  // a scaled area based on two factors:
  //   - We don't need the actual area; any non-zero scale of the area would
  //     work just fine.
  //   - the normal (nhat_F) while documented as unit length may not actually
  //     be. However, for a planar polygon, each decomposed triangle should
  //     produce a parallel cross product and dotting each of those with an
  //     _arbitrary_ normal direction simply further scales the area by
  //     another constant, leaving the relative weights the same.
  auto triangle_weight = [&vertices_F, &nhat_F](V v0, V v1, V v2) {
    const Vector3<T>& r_MV0 = vertices_F[v0].r_MV();
    const Vector3<T>& r_MV1 = vertices_F[v1].r_MV();
    const Vector3<T>& r_MV2 = vertices_F[v2].r_MV();
    return (r_MV1 - r_MV0).cross(r_MV2 - r_MV0).dot(nhat_F);
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

  return p_FC_accum / total_weight;
}

template <typename T>
void AddPolygonToMeshData(
    const std::vector<SurfaceVertexIndex>& polygon,
    const Vector3<T>& nhat_F,
    std::vector<SurfaceFace>* faces,
    std::vector<SurfaceVertex<T>>* vertices_F) {
  DRAKE_DEMAND(faces != nullptr);
  DRAKE_DEMAND(vertices_F != nullptr);
  DRAKE_DEMAND(polygon.size() >= 3);

  // We're going to create a triangle fan for the polygon. This requires adding
  // a new vertex at the polygon's geometric centroid (which is the centroid
  // for this "uniform material" polygon).
  Vector3<T> p_FC = CalcPolygonCentroid(polygon, nhat_F, *vertices_F);
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
    const Vector3<double>& nhat_F,
    const std::vector<SurfaceVertex<double>>& vertices_F);

template
void AddPolygonToMeshData(
    const std::vector<SurfaceVertexIndex>& polygon,
    const Vector3<double>& nhat_F,
    std::vector<SurfaceFace>* faces,
    std::vector<SurfaceVertex<double>>* vertices_F);
    
}  // namespace internal
}  // namespace geometry
}  // namespace drake
