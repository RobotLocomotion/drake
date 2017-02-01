#include "drake/systems/sensors/camera_info.h"

namespace drake {
namespace systems {
namespace sensors {

CameraInfo::CameraInfo(int width, int height, double vertical_fov_rad)
    : CameraInfo(width, height, width * 0.5 / std::tan(
          0.5 * width / height * vertical_fov_rad),
                 height * 0.5 / std::tan(0.5 * vertical_fov_rad),
                 width * 0.5, height * 0.5) {}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
