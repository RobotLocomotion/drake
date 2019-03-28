#include "drake/geometry/query_results/contact_surface.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace geometry {

// TODO(DamrongGuoy): Get the line below working.
//   DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
//    class ContactSurface);

template <class T>
T SurfaceMesh<T>::interpolate(const MeshField<T, VertexIndex>& field,
                              FaceIndex f, const Vector3<T>& barycentric) {
  T value(0);
  for (int i = 0; i < 3; ++i) {
    value += barycentric(i) * field.at(faces_[f].vertex(i));
  }
  return value;
}

template <class T>
T ContactSurface<T>::e(FaceIndex f, const Vector3<T>& barycentric) {
  return mesh_.interpolate(e_, f, barycentric);
}

template <class T>
Vector3<T> ContactSurface<T>::grad_h_M(FaceIndex f, const Vector3<T>&
    barycentric) {
  // TODO(DamrongGuoy): Switch to spherical linear interpolation.
  return mesh_.interpolate(grad_h_M_, f, barycentric);
}

}  // namespace geometry
}  // namespace drake

