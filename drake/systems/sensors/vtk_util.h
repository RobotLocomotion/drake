#pragma once

#include <Eigen/Dense>
#include <vtkPlaneSource.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace systems {
namespace sensors {

/// Utility class for the VTK library.
class VtkUtil {
 public:
  VtkUtil() = delete;
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VtkUtil)

  /// Creates a square plane of the given size.  The plane's z axis is its
  /// normal and is coincident with the z axis of the world coordinate system.
  /// The plane's center is at (0, 0, 0) of both the world coordinate system and
  /// plane's coordinate system.  The ranges of the plane's x and y are
  /// `(- size / 2, size / 2)`.
  ///
  /// @param size The size of the plane.
  static vtkSmartPointer<vtkPlaneSource> CreateSquarePlane(double size);

  /// Converts the provided `Eigen::Isometry3d` to a `vtkTransform`.
  ///
  /// @param transform The transform to convert into a `vtkTransform`.
  static vtkSmartPointer<vtkTransform> ConvertToVtkTransform(
      const Eigen::Isometry3d& transform);
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
