#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/systems/sensors/rgbd_renderer_vtk.h"

namespace drake {
namespace systems {
namespace sensors {

using RgbdCameraVTK = RgbdCamera<RgbdRendererVTK>;
using RgbdCameraDiscreteVTK = RgbdCameraDiscrete<RgbdRendererVTK>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
