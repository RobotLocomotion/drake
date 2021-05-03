#include "drake/math/rotation_matrix.h"

#include <string>

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
          "  Measure of orthonormality error: {}  (near-zero is good).\n"
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

template <typename T>
void RotationMatrix<T>::ThrowIfNotUnitLength(const Vector3<T>& v,
                                             const char* function_name) {
  if constexpr (scalar_predicate<T>::is_bool) {
    // The value of kEps was determined empirically, is well within the
    // tolerance achieved by normalizing a vast range of non-zero vectors, and
    // seems to guarantee a valid RotationMatrix() (see IsValid()).
    constexpr double kEps = 4 * std::numeric_limits<double>::epsilon();
    const double norm = ExtractDoubleOrThrow(v.norm());
    const double error = std::abs(1.0 - norm);
    // If v contains non-finite values (NaN or infinity), this test must fail.
    // Note: To properly test for Nan and infinity, do not change the next
    // logic statement to if( error > kEps).
    if ((error <= kEps) == false) {
      const double vx = ExtractDoubleOrThrow(v.x());
      const double vy = ExtractDoubleOrThrow(v.y());
      const double vz = ExtractDoubleOrThrow(v.z());
      throw std::logic_error(
        fmt::format("RotationMatrix::{}() requires a unit-length vector.\n"
                    "         v: {} {} {}\n"
                    "       |v|: {}\n"
                    " |1 - |v||: {} is not less than or equal to {}.",
                    function_name, vx, vy, vz, norm, error, kEps));
    }
  } else {
    unused(v, function_name);
  }
}

template <typename T>
Vector3<T> RotationMatrix<T>::NormalizeOrThrow(const Vector3<T>& v,
                                               const char* function_name) {
  Vector3<T> u;
  if constexpr (scalar_predicate<T>::is_bool) {
    // The number 1.0E-10 is a heuristic (rule of thumb) that is guided by
    // an expected small physical dimensions in a robotic systems.  Numbers
    // smaller than this are probably user or developer errors.
    constexpr double kMinMagnitude = 1e-10;
    constexpr double kInf = std::numeric_limits<double>::infinity();
    const double norm = ExtractDoubleOrThrow(v.norm());
    // If v contains non-finite values (NaN or infinity), this test must fail.
    // For IEEE 754, except NaN and infinity, everything is less than infinity.
    if (norm >= kMinMagnitude && norm < kInf) {
      u = v/norm;
    } else {
      const double vx = ExtractDoubleOrThrow(v.x());
      const double vy = ExtractDoubleOrThrow(v.y());
      const double vz = ExtractDoubleOrThrow(v.z());
      throw std::logic_error(
        fmt::format("RotationMatrix::{}() cannot normalize the given vector.\n"
                    "   v: {} {} {}\n"
                    " |v|: {}\n"
                    " The measures must be finite and the vector must have a"
                    " magnitude of at least {} to automatically normalize. If"
                    " you are confident that v's direction is meaningful, pass"
                    " v.normalized() in place of v.",
                    function_name, vx, vy, vz, norm, kMinMagnitude));
    }
  } else {
    unused(function_name);
    u = v.normalized();
  }
  return u;
}

}  // namespace math
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::math::RotationMatrix)
