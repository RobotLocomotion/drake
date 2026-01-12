#include "drake/geometry/render_vtk/internal_make_render_window.h"

// These include files incorporate `#define ...` madness that causes
// RenderEngineVtk to not even compile anymore because some keyword(s)
// and/or identifier(s) have been replaced, so we must segregate this
// helper function into its own little translation unit.

// To ease build system upkeep, we annotate VTK includes with their deps.
#if defined(__APPLE__)
#include <vtkCocoaRenderWindow.h>  // vtkRenderingOpenGL2
#else
#include <vtkEGLRenderWindow.h>      // vtkRenderingOpenGL2
#include <vtkX11Functions.h>         // vtkx11
#include <vtkXOpenGLRenderWindow.h>  // vtkRenderingOpenGL2
#endif

#include "drake/geometry/render_gl/internal_loaders.h"

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {

// The list of what's available on Linux and Apple here must be kept in sync
// with the logic in render_engine_vtk_params.cc.
vtkSmartPointer<vtkRenderWindow> MakeRenderWindow(
    RenderEngineVtkBackend backend) {
  switch (backend) {
    case RenderEngineVtkBackend::kCocoa: {
#if defined(__APPLE__)
      return vtkSmartPointer<vtkCocoaRenderWindow>::New();
#else
      break;
#endif
    }
    case RenderEngineVtkBackend::kEgl: {
#if !defined(__APPLE__)
      render_gl::internal::GladLoaderLoadEgl();
      return vtkSmartPointer<vtkEGLRenderWindow>::New();
#else
      break;
#endif
    }
    case RenderEngineVtkBackend::kGlx: {
#if !defined(__APPLE__)
      Display* display =
          static_cast<Display*>(render_gl::internal::GladLoaderLoadGlx());
      DRAKE_DEMAND(display != nullptr);
      auto result = vtkSmartPointer<vtkXOpenGLRenderWindow>::New();
      result->SetDisplayId(display);
      return result;
#else
      break;
#endif
    }
  }
  // This is not reachable via Drake's public API. The params parsing logic
  // checks for platform-vs-backend compatibility.
  throw std::logic_error("MakeRenderWindow(backend=...) is not available");
}

}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
