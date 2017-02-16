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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VtkUtil)

  /// Creates a square plane with the given size.  The plane's z axis is its
  /// normal and is coincident with the z axis of the world coordinate systesm.
  /// The plane's center is at (0, 0, 0).  The ranges of the plane's x and y are
  /// `(- size / 2, size / 2)`.
  ///
  /// @param size The size of the plane.
  static vtkSmartPointer<vtkPlaneSource> CreateSquarePlane(double size);

  /// Converts the transformation of Eigen::Isometry3d to that of vtkTransform.
  ///
  /// @param transform The transformation of Eigen::Isometry3d.
  static vtkSmartPointer<vtkTransform> ConvertToVtkTransform(
      const Eigen::Isometry3d& transform);
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
