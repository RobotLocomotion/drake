#include "drake/geometry/proximity/polygon_surface_mesh.h"

#include <limits>

namespace drake {
namespace geometry {

using math::RigidTransform;
using std::move;
using std::vector;

std::unique_ptr<SurfacePolygon> SurfacePolygon::copy_to_unique() const {
  return std::unique_ptr<SurfacePolygon>(new SurfacePolygon(
      &mesh_face_data_, index_));
}

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
    n = (X_NM.rotation() * n).normalized();
  }
  for (auto& c : element_centroid_M_) {
    c = X_NM * c;
  }
  p_MSc_ = X_NM * p_MSc_;
}

template <typename T>
std::pair<Vector3<T>, Vector3<T>> PolygonSurfaceMesh<T>::CalcBoundingBox()
    const {
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

  Vector3<T> normal_M{0, 0, 0};
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
   0. We use the normal of the last triangle in the fan as the face normal; this
   only works because of the requirement that the polygons be strictly planar
   and convex.

   For efficiency, we actually accumulate 2 * area in double_face_area and
   6 * p_MTc in p_MTc_scaled to defer otherwise redundant scaling operations. */
  for (int v = 1; v < v_count - 1; ++v) {
    const Vector3<T>& r_MB = vertices_M_[face_data_[v_index_offset + v]];
    const Vector3<T>& r_MC = vertices_M_[face_data_[v_index_offset + v + 1]];
    const auto r_UV_M = r_MB - r_MA;
    const auto r_UW_M = r_MC - r_MA;

    cross = r_UV_M.cross(r_UW_M);
    normal_M += cross;
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
     the final value.

     The area value used here is an *absolute* area value. This requires the
     polygon to be convex (although it does allow for collinear vertices). To
     allow for non-convex polygons, we'd have to have a *signed* area. This is
     most easily done by using a known normal direction for computing the
     area. */
    p_MTc_scaled += cross_magnitude * (r_MA + r_MB + r_MC);
  }

  /* Correct the scaled poly centroid as documented above. */
  const T poly_area = double_poly_area / 2;
  areas_.push_back(poly_area);

  /* We've accumulated all of the cross products into normal_M. The resulting
   vector result is the area-scaled polygon normal. This vector sum protects us
   from any triangles in the fan that may have zero area, or even if the
   triangle is non-convex. However, this robust *normal* calculation still
   does not allow this function to support non-convex polygons (see the note
   on computing the polygon centroid). If the area of the polygon is properly
   zero, we rely on Eigen's documented behavior that the normalized zero vector
   is the zero vector.
   https://eigen.tuxfamily.org/dox/classEigen_1_1MatrixBase.html#a5cf2fd4c57e59604fd4116158fd34308
   */
  face_normals_.emplace_back(normal_M.normalized());

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

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class PolygonSurfaceMesh)

}  // namespace geometry
}  // namespace drake
