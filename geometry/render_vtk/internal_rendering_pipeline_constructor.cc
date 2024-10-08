#include "drake/geometry/render_vtk/internal_render_engine_vtk.h"

// These include files incorporate `#define ...` madness that causes
// RenderEngineVtk to not even compile anymore because some keyword(s)
// and/or identifier(s) have been replaced, so we must segregate this
// constructor body into its own little translation unit.

#if defined(__APPLE__)
#include "vtkCocoaRenderWindow.h"
#else
#include "vtkXOpenGLRenderWindow.h"
#include "vtkglad/include/glad/glx.h"
#endif

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {

RenderEngineVtk::RenderingPipeline::RenderingPipeline() {
#if defined(__APPLE__)
  window = vtkSmartPointer<vtkCocoaRenderWindow>::New();
#else
  gladLoaderLoadGLX(nullptr, 0);
  window = vtkSmartPointer<vtkXOpenGLRenderWindow>::New();
#endif
  DRAKE_DEMAND(window != nullptr);
}

}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
