#include "drake/geometry/render_vtk/internal_make_render_window.h"

// These include files incorporate `#define ...` madness that causes
// RenderEngineVtk to not even compile anymore because some keyword(s)
// and/or identifier(s) have been replaced, so we must segregate this
// helper function into its own little translation unit.

// To ease build system upkeep, we annotate VTK includes with their deps.
#if defined(__APPLE__)
#include <vtkCocoaRenderWindow.h>  // vtkRenderingOpenGL2
#else
#include <vtkEGLRenderWindow.h>        // vtkRenderingOpenGL2
#include <vtkXOpenGLRenderWindow.h>    // vtkRenderingOpenGL2
#include <vtkglad/include/glad/egl.h>  // vtkglad
#include <vtkglad/include/glad/glx.h>  // vtkglad
#endif

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/unused.h"

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {

// The list of what's available on Linux and Apple here must be kept in sync
// with the logic in render_engine_vtk_params.cc.
vtkSmartPointer<vtkRenderWindow> MakeRenderWindow(
    RenderEngineVtkBackend backend) {
  vtkSmartPointer<vtkRenderWindow> result;
  switch (backend) {
    case RenderEngineVtkBackend::kCocoa: {
#if defined(__APPLE__)
      return vtkSmartPointer<vtkCocoaRenderWindow>::New();
#endif
    }
    case RenderEngineVtkBackend::kEgl: {
#if !defined(__APPLE__)
      // Open the library at most once per process. At the time of this
      // writing this does not appear to matter in practice, but we might as
      // well avoid any problems down the road.
      static const bool load_egl_success = gladLoaderLoadEGL(EGL_NO_DISPLAY);
      DRAKE_THROW_UNLESS(load_egl_success);
      return vtkSmartPointer<vtkEGLRenderWindow>::New();
#endif
    }
    case RenderEngineVtkBackend::kGlx: {
#if !defined(__APPLE__)
      // Open the library at most once per process. This is important because
      // loading the library every time leaks resources ("too many clients")
      // that are in short supply when using a typical Xorg server.
      static const int kVersion = gladLoaderLoadGLX(nullptr, 0);
      unused(kVersion);
      return vtkSmartPointer<vtkXOpenGLRenderWindow>::New();
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
