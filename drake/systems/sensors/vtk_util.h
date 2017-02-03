#pragma once

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

namespace drake {
namespace systems {
namespace sensors {

/// Utility class for the VTK library.
class VtkUtil {
 public:
  /// Creates a square plane with the given size, color and:
  ///   - the plane's axes are coincident with those of the world coordinate
  ///     system
  ///   - the plane's center is at (0, 0, 0) of the world coordinate system.
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
