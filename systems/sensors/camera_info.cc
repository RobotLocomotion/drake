#include "drake/systems/sensors/camera_info.h"

#include <cmath>
#include <sstream>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/fmt_eigen.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {

constexpr char kPrefix[] = "\n  ";

template <typename T>
void FormatNVP(const char* name, T value, std::stringstream* s) {
  (*s) << kPrefix << name << " (" << value << ")";
}

void CheckPositive(const char* name, int value, std::stringstream* s) {
  if (value <= 0) {
    FormatNVP(name, value, s);
    (*s) << " should be positive.";
  }
}

void CheckBetweenZeroAndMax(const char* name, double value, int max,
                            std::stringstream* s) {
  if (value <= 0 || value >= static_cast<double>(max)) {
    FormatNVP(name, value, s);
    (*s) << " should lie in the range (0, " << max << ").";
  }
}

void CheckPositiveFinite(const char* name, double value, std::stringstream* s) {
  if (value <= 0 || !std::isfinite(value)) {
    FormatNVP(name, value, s);
    (*s) << " should be a positive, finite number.";
  }
}

}  // namespace

CameraInfo::CameraInfo(int width, int height, double focal_x, double focal_y,
             double center_x, double center_y)
    : CameraInfo(width, height, (
          Eigen::Matrix3d() << focal_x, 0., center_x,
                               0., focal_y, center_y,
                               0., 0., 1.).finished()) {}

CameraInfo::CameraInfo(
      int width, int height, const Eigen::Matrix3d& intrinsic_matrix)
    : width_(width), height_(height),
      intrinsic_matrix_(intrinsic_matrix) {
  std::stringstream errors;

  CheckPositive("Width", width, &errors);
  CheckPositive("Height", height, &errors);

  const Eigen::Matrix3d& K = intrinsic_matrix;
  CheckPositiveFinite("Focal X", K(0, 0), &errors);
  CheckPositiveFinite("Focal Y", K(1, 1), &errors);
  CheckBetweenZeroAndMax("Center X", K(0, 2), width, &errors);
  CheckBetweenZeroAndMax("Center Y", K(1, 2), height, &errors);

  // Off-diagonal terms should be zero and homogeneous row should be [0, 0, 1].
  // TODO(eric.cousineau): Relax this with a tolerance?
  if (K(0, 1) != 0 || K(1, 0) != 0 || K(2, 0) != 0 || K(2, 1) != 0 ||
      K(2, 2) != 1) {
    errors << kPrefix << "The camera's intrinsic matrix is malformed:\n"
           << fmt::to_string(fmt_eigen(K));
  }

  const std::string error_message = errors.str();
  if (!error_message.empty()) {
    throw std::runtime_error("Invalid camera configuration: " + error_message);
  }
}

CameraInfo::CameraInfo(int width, int height, double vertical_fov_rad)
    : CameraInfo(width, height,
                 height * 0.5 / std::tan(0.5 * vertical_fov_rad),
                 height * 0.5 / std::tan(0.5 * vertical_fov_rad),
                 width * 0.5 - 0.5, height * 0.5 - 0.5) {}
// TODO(SeanCurtis-TRI): The shift of the principal point by (-0.5, -0.5) is
//  overly opaque. The primary explanation comes from pixel addressing
//  conventions used in various APIs (see
//  https://www.khronos.org/registry/OpenGL/extensions/ARB/ARB_fragment_coord_conventions.txt
//  However, we don't want to look like this class is coupled with OpenGL. How
//  do we articulate this math in a way that *doesn't* depend on OpenGL?
//  See https://github.com/SeanCurtis-TRI/drake/pull/5#pullrequestreview-264447958.

}  // namespace sensors
}  // namespace systems
}  // namespace drake
