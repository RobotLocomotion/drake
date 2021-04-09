#include "drake/math/rotation_matrix.h"

#include <fmt/format.h>

#include "drake/common/unused.h"

namespace drake {
namespace math {

template <typename T>
void RotationMatrix<T>::ThrowIfNotValid(const Matrix3<T>& R) {
  if constexpr (scalar_predicate<T>::is_bool) {
    if (!R.allFinite()) {
      throw std::logic_error(
          "Error: Rotation matrix contains an element that is infinity or"
          " NaN.");
    }
    // If the matrix is not-orthogonal, try to give a detailed message.
    // This is particularly important if matrix is very-near orthogonal.
    if (!IsOrthonormal(R, get_internal_tolerance_for_orthonormality())) {
      const T measure_of_orthonormality = GetMeasureOfOrthonormality(R);
      const double measure = ExtractDoubleOrThrow(measure_of_orthonormality);
      std::string message = fmt::format(
          "Error: Rotation matrix is not orthonormal.\n"
          "  Measure of orthonormality error: {:G}  (near-zero is good).\n"
          "  To calculate the proper orthonormal rotation matrix closest to"
          " the alleged rotation matrix, use the SVD (expensive) static method"
          " RotationMatrix<T>::ProjectToRotationMatrix(), or for a less"
          " expensive (but not necessarily closest) rotation matrix, use"
          " RotationMatrix<T>(RotationMatrix<T>::ToQuaternion<T>(your_matrix))."
          " Alternatively, if using quaternions, ensure the quaternion is"
          " normalized.", measure);
      throw std::logic_error(message);
    }
    if (R.determinant() < 0) {
      throw std::logic_error(
          "Error: Rotation matrix determinant is negative."
          " It is possible a basis is left-handed.");
    }
  } else {
    unused(R);
  }
}

double ProjectMatToRotMatWithAxis(const Eigen::Matrix3d& M,
                                  const Eigen::Vector3d& axis,
                                  const double angle_lb,
                                  const double angle_ub) {
  if (angle_ub < angle_lb) {
    throw std::runtime_error(
        "The angle upper bound should be no smaller than the angle lower "
        "bound.");
  }
  const double axis_norm = axis.norm();
  if (axis_norm == 0) {
    throw std::runtime_error("The axis argument cannot be the zero vector.");
  }
  const Eigen::Vector3d a = axis / axis_norm;
  Eigen::Matrix3d A;
  // clang-format off
  A <<    0,  -a(2),   a(1),
       a(2),      0,  -a(0),
      -a(1),   a(0),      0;
  // clang-format on
  const double alpha =
      atan2(-(M.transpose() * A * A).trace(), (A.transpose() * M).trace());
  double theta{};
  // The bounds on θ + α is [angle_lb + α, angle_ub + α].
  if (std::isinf(angle_lb) && std::isinf(angle_ub)) {
    theta = M_PI_2 - alpha;
  } else if (std::isinf(angle_ub)) {
    // First if the angle upper bound is inf, start from the angle_lb, and
    // find the angle θ, such that θ + α = 0.5π + 2kπ
    const int k = ceil((angle_lb + alpha - M_PI_2) / (2 * M_PI));
    theta = (2 * k + 0.5) * M_PI - alpha;
  } else if (std::isinf(angle_lb)) {
    // If the angle lower bound is inf, start from the angle_ub, and find the
    // angle θ, such that θ + α = 0.5π + 2kπ
    const int k = floor((angle_ub + alpha - M_PI_2) / (2 * M_PI));
    theta = (2 * k + 0.5) * M_PI - alpha;
  } else {
    // Now neither angle_lb nor angle_ub is inf. Check if there exists an
    // integer k, such that 0.5π + 2kπ ∈ [angle_lb + α, angle_ub + α]
    const int k = floor((angle_ub + alpha - M_PI_2) / (2 * M_PI));
    const double max_sin_angle = M_PI_2 + 2 * k * M_PI;
    if (max_sin_angle >= angle_lb + alpha) {
      // 0.5π + 2kπ ∈ [angle_lb + α, angle_ub + α]
      theta = max_sin_angle - alpha;
    } else {
      // Now the maximal is at the boundary, either θ = angle_lb or angle_ub
      if (sin(angle_lb + alpha) >= sin(angle_ub + alpha)) {
        theta = angle_lb;
      } else {
        theta = angle_ub;
      }
    }
  }
  return theta;
}

}  // namespace math
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::math::RotationMatrix)
