#pragma once

#include "drake/geometry/render_vtk/render_engine_vtk_params.h"

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkRenderWindow.h>  // vtkRenderingCore

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {

/* Returns a newly-constructed vtkRenderWindow, or else throws when unable. */
vtkSmartPointer<vtkRenderWindow> MakeRenderWindow(
    RenderEngineVtkBackend backend);

}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
