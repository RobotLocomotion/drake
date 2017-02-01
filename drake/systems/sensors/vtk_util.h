#pragma once

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

namespace drake {
namespace systems {
namespace sensors {

/// Utility class for VTK library.
class VtkUtil {
 public:
  /// Create a square plane with the given size, color and:
  ///   - the z axis to be the plane normal
  ///   - the plane center at (0, 0, 0).
  ///
  /// @param size The square size.
  /// @param color The color of the plane.  The order of the values is red,
  /// green and blue.
  static vtkSmartPointer<vtkPolyData> CreateSquarePlane(
      double size, const unsigned char color[3]);
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
