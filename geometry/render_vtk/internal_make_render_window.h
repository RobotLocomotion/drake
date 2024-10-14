#pragma once

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkRenderWindow.h>  // vtkRenderingCore

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {

/* Returns a newly-constructed vtkRenderWindow, or else throws when unable.
On Linux, use_egl chooses between EGL (when true) and GLX (when false).
On macOS, use_egl is ignored. */
vtkSmartPointer<vtkRenderWindow> MakeRenderWindow(bool use_egl);

}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
