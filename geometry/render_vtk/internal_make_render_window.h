#pragma once

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkRenderWindow.h>  // vtkRenderingCore

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {

vtkSmartPointer<vtkRenderWindow> MakeRenderWindow();

}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
