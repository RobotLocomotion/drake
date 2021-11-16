#include "drake/geometry/proximity/polygon_surface_mesh.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {

using math::RigidTransform;
using std::move;
using std::vector;

template <typename T>
PolygonSurfaceMesh<T>::PolygonSurfaceMesh(vector<int> face_data,
                                          vector<Vector3<T>> vertices)
    : face_data_(move(face_data)),
      vertices_M_(move(vertices)),
      p_MSc_(Vector3<T>::Zero()) {
  /* Build the polygons and derived quantities from the given data. */
  int poly_count = -1;
  int i = 0;
  while (i < static_cast<int>(face_data_.size())) {
    poly_indices_.push_back(i);
    CalcAreaNormalAndCentroid(++poly_count);
    i += face_data_[i] + 1;  /* Jump to the next polygon. */
  }
  DRAKE_DEMAND(poly_indices_.size() == areas_.size());
  DRAKE_DEMAND(poly_indices_.size() == face_normals_.size());
}

template <typename T>
void PolygonSurfaceMesh<T>::TransformVertices(const RigidTransform<T>& X_NM) {
  for (auto& v : vertices_M_) {
    v = X_NM * v;
  }
  for (auto& n : face_normals_) {
    n = X_NM.rotation() * n;
  }
  for (auto& c : element_centroid_M_) {
    c = X_NM * c;
  }
  p_MSc_ = X_NM * p_MSc_;
}

template <typename T>
void PolygonSurfaceMesh<T>::ReverseFaceWinding() {
  for (const int f_index : poly_indices_) {
    const int v_count = face_data_[f_index];
    /* The indices before and after the first and last entries.  */
    int f_0 = f_index;
    int f_n = f_index + v_count + 1;
    for (int i = 0; i < v_count / 2; ++i) {
      std::swap(face_data_[++f_0], face_data_[--f_n]);
    }
  }
  for (auto& n : face_normals_) {
    n = -n;
  }
}

template <typename T>
std::pair<Vector3<T>, Vector3<T>> PolygonSurfaceMesh<T>::CalcBoundingBox()
    const {
  /* TODO(-TRI): Are we actually using this?  */
  Vector3<T> min_extent =
      Vector3<T>::Constant(std::numeric_limits<double>::max());
  Vector3<T> max_extent =
      Vector3<T>::Constant(std::numeric_limits<double>::lowest());
  for (const auto& vertex : vertices_M_) {
    min_extent = min_extent.cwiseMin(vertex);
    max_extent = max_extent.cwiseMax(vertex);
  }
  Vector3<T> center = (max_extent + min_extent) / 2.0;
  Vector3<T> size = max_extent - min_extent;
  return std::make_pair(center, size);
}

template <typename T>
bool PolygonSurfaceMesh<T>::Equal(const PolygonSurfaceMesh<T>& mesh) const {
  if (this == &mesh) return true;

  if (this->num_faces() != mesh.num_faces()) return false;
  if (this->num_vertices() != mesh.num_vertices()) return false;
  if (this->vertices_M_ != mesh.vertices_M_) return false;
  if (this->poly_indices_ != mesh.poly_indices_) return false;
  if (this->face_data_ != mesh.face_data_) return false;

  return true;
}

template <class T>
void PolygonSurfaceMesh<T>::CalcAreaNormalAndCentroid(int poly_index) {
  int data_index = poly_indices_[poly_index];
  const int v_count = face_data_[data_index];
  int v_index_offset = data_index + 1;
  const Vector3<T>& r_MA = vertices_M_[face_data_[v_index_offset]];

  Vector3<T> p_MTc_scaled(0, 0, 0);

  Vector3<T> cross;
  T cross_magnitude{};
  T double_poly_area(0);

  // TODO(SeanCurtis-TRI): We could reduce square roots by computing the normal
  //  from the first triangle and use that to facilitate area calculations for
  //  subsequent triangles. This *kind* of would allow this algorithm to work
  //  with non-convex polygons. However, where it fails is if the first triangle
  //  has locally reversed winding. The normal direction would not be as
  //  expected. The best solution is to pass in normals upon construction (we
  //  have them in computing a contact surface) and use those normals to compute
  //  area.

  /* Compute the polygon *total* area and centroid (Tc) by decomposing it into a
   set of triangles. The triangles are defined as a triangle fan around vertex
   0. This only works because of the requirement that the polygons be strictly
   planar and convex.

   For efficiency, we actually accumulate 2 * area in double_face_area and
   6 * p_MTc in p_MTc_scaled to defer otherwise redundant scaling operations. */
  for (int v = 1; v < v_count - 1; ++v) {
    const Vector3<T>& r_MB = vertices_M_[face_data_[v_index_offset + v]];
    const Vector3<T>& r_MC = vertices_M_[face_data_[v_index_offset + v + 1]];
    const auto r_UV_M = r_MB - r_MA;
    const auto r_UW_M = r_MC - r_MA;

    cross = r_UV_M.cross(r_UW_M);
    /* The cross product magnitude is equal to twice the triangle area. */
    cross_magnitude = cross.norm();
    double_poly_area += cross_magnitude;

    /* The triangle centroid Tc is the mean position of the vertices:
     (A + B + C) / 3. Its contribution to the polygon centroid is scaled by
     its portion of the polygon area: Aₜ⋅(A + B + C) / (3⋅Aₚ) (where Aₜ is the
     triangle area and Aₚ is the polygon area). Because we are doing this
     weighted sum, it doesn't matter if the scale factor is Aₜ / (3⋅Aₚ) or
     k⋅Aₜ / (k⋅3⋅Aₚ), the scale factors cancel out. So, that means we can ignore
     the factor of two in the "double" area and omit the divisor 3 in the mean
     vertex position when combining triangle centroids; *proportional*
     contribution remains the same. We'll deal with this factor 6 when we need
     the final value. */
    p_MTc_scaled += cross_magnitude * (r_MA + r_MB + r_MC);
  }

  /* Correct the scaled poly centroid as documented above. */
  const T poly_area = double_poly_area / 2;
  areas_.push_back(poly_area);

  /* Finally, given the requirement that the polygon is planar and convex, the
   normal direction of the last triangle can represent the normal direction
   for the polygon. We only normalize for strictly non-zero cross products.
   The direction of the normal for very small cross-product values may or may
   not be meaningful based on the size of the triangle or the aspect ratio. We
   are not currently trying to detect and correct bad normal directions. */
  face_normals_.emplace_back(cross_magnitude != T(0) ? cross / cross_magnitude
                                                     : cross);

  /* Accumulate the contribution of *this* polygon into the mesh centroid. We
   implicitly have the weighted contribution of all previous polygons as the
   product of current centroid and total area. This allows us to add in this
   polygon's contribution proportionately (although, we want to add in Aₚ⋅p_MFc,
   but what we computed is 6⋅Aₚ⋅p_MFc. */
  const Vector3<T> p_MTc_area_scaled = p_MTc_scaled / 6;
  element_centroid_M_.emplace_back(p_MTc_area_scaled / poly_area);
  const T old_area = total_area_;
  total_area_ += poly_area;
  p_MSc_ = (p_MSc_ * old_area + p_MTc_area_scaled) / total_area_;
}

template <typename T>
Vector3<T> PolygonSurfaceMesh<T>::CalcGradBarycentric(
    const Vector3<T>& p_MV, const Vector3<T>& p_MA,
    const Vector3<T>& p_MB) const {
  const Vector3<T> p_AB_M = p_MB - p_MA;
  const T ab2 = p_AB_M.squaredNorm();
  const Vector3<T> p_AV_M = p_MV - p_MA;
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  constexpr double kEps2 = kEps * kEps;
  // For a skinny triangle with the edge AB that has zero or almost-zero
  // length, we set the vector H to be AV.
  const Vector3<T>& H_M =
      (ab2 <= kEps2) ? p_AV_M : p_AV_M - (p_AV_M.dot(p_AB_M) / ab2) * p_AB_M;
  const T h2 = H_M.squaredNorm();
  if (h2 <= kEps2) {
    throw std::runtime_error("Bad triangle. Cannot compute gradient.");
  }
  return H_M / h2;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class PolygonSurfaceMesh)

}  // namespace geometry
}  // namespace drake
