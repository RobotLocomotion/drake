#include "drake/systems/sensors/camera_info.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace sensors {

CameraInfo::CameraInfo(int width, int height, double focal_x, double focal_y,
             double center_x, double center_y)
    : width_(width), height_(height), intrinsic_matrix_((
          Eigen::Matrix3d() << focal_x, 0., center_x,
                               0., focal_y, center_y,
                               0., 0., 1.).finished()) {
  DRAKE_ASSERT(width > 0);
  DRAKE_ASSERT(height > 0);
  DRAKE_ASSERT(focal_x > 0);
  DRAKE_ASSERT(focal_y > 0);
  DRAKE_ASSERT(center_x > 0 && center_x < static_cast<double>(width));
  DRAKE_ASSERT(center_y > 0 && center_y < static_cast<double>(height));
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
