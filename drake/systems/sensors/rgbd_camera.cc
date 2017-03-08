#include "drake/systems/sensors/rgbd_camera.h"

#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

namespace drake {
namespace systems {
namespace sensors {

// TODO(kunimatsu.hashimoto) Implement this.
RgbdCamera::RgbdCamera() {
  vtkSmartPointer<vtkRenderer> renderer =
      vtkSmartPointer<vtkRenderer>::New();
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
