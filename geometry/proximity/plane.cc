#include "drake/geometry/proximity/plane.h"

#include "drake/common/drake_assert.h"
#include "drake/common/fmt_eigen.h"

namespace drake {
namespace geometry {

using Eigen::Vector3d;
using math::RotationMatrixd;

template <typename T>
Plane<T>::Plane(const Vector3<T>& normal, const Vector3<T>& point_on_plane,
                bool already_normalized) {
  const Vector3<T>& n_F = normal;
  const Vector3<T>& p_FP = point_on_plane;
  if (!already_normalized) {
    const T magnitude = n_F.norm();
    // NOTE: This threshold is arbitrary. Given Drake uses mks and generally
    // works on problems at a human scale, the assumption is that if someone
    // passes in an incredibly small normal (picometers), it is probably an
    // error.
    if (magnitude < 1e-10) {
      throw std::runtime_error(fmt::format(
          "Cannot instantiate plane from normal n_F = [{}]; its magnitude is "
          "too small: {}",
          fmt_eigen(n_F.transpose()), magnitude));
    }
    nhat_F_ = n_F / magnitude;
  } else {
    DRAKE_ASSERT_VOID(ThrowIfInsufficientlyNormal(n_F));
    nhat_F_ = n_F;
  }
  displacement_ = nhat_F_.dot(p_FP);
}

template <typename T>
bool Plane<T>::BoxOverlaps(
    const Vector3d& half_width, const Vector3d& box_center_in_plane,
    const RotationMatrixd& box_orientation_in_plane) const {
  /* The box doesn't overlap the plane if the box center is sufficiently far
   from the plane to provide "clearance". If we consider the vector from the
   "lowest" (defined relative to the normal direction) point on the box L to the
   box origin Bo, p_LBo, the measure of that vector's projection onto the plane
   normal, p_LBo·n_P, is exactly the clearance distance.

   By definition, the vector p_LBo = ±Bx·hx + ±By·hy + ±Bz·hz, where the
   signs applied to the basis vectors depend on the relative orientation of B
   and P. Specifically, we'd pick the signs of the basis vectors so that Bi·n_P
   is non-negative. Fortunately, we don't need to find the actual point L; we
   only need the measure of the projected vector. We can exploit the fact that
   (-Bi)·n_P = -(Bi·n_P); |Bi·n_P| is equal to ±Bi·n_P where we've picked
   the "right" sign. */

  const RotationMatrixd& R_PB = box_orientation_in_plane;
  double half_extent_along_normal = 0.0;
  const Vector3d& n_P = ExtractDoubleOrThrow(unit_normal());
  for (int i = 0; i < 3; ++i) {
    const Vector3d& Bi_P = R_PB.col(i);
    double extent = std::abs(Bi_P.dot(n_P)) * half_width(i);
    half_extent_along_normal += extent;
  }

  const Vector3d& p_PoBo_P = box_center_in_plane;
  const double box_center_height = ExtractDoubleOrThrow(CalcHeight(p_PoBo_P));
  return std::abs(box_center_height) <= half_extent_along_normal;
}

template <typename T>
void Plane<T>::ThrowIfInsufficientlyNormal(const Vector3<T>& n) {
  using std::abs;
  const T delta = abs(n.norm() - 1);
  // We pick a threshold that should easily contain typical precision loss
  // in the columns/rows of a rotation matrix -- a likely source for plane
  // normals.
  if (delta > 1e-13) {
    throw std::runtime_error(fmt::format(
        "Plane constructed with a normal vector that was declared normalized;"
        " the vector is not unit length. Vector [{}] with length {}",
        fmt_eigen(n.transpose()), T{n.norm()}));
  }
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class Plane);

}  // namespace geometry
}  // namespace drake
