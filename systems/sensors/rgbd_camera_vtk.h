#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/systems/sensors/rgbd_renderer_vtk.h"

namespace drake {
namespace systems {
namespace sensors {

/// RgbdCameraVTK and RgbdCameraDiscreteVTK uses
/// [VTK](https://github.com/Kitware/VTK) as the rendering backend.
/// @ingroup sensor_systems
using RgbdCameraVTK = RgbdCamera<RgbdRendererVTK>;
using RgbdCameraDiscreteVTK = RgbdCameraDiscrete<RgbdRendererVTK>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
