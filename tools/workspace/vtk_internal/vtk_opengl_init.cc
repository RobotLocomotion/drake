#include "drake/tools/workspace/vtk_internal/vtk_opengl_init.h"

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkAutoInit.h>  // vtkCommonCore

VTK_AUTOINIT_DECLARE(vtkRenderingOpenGL2)

namespace drake {
namespace internal {

void VtkOpenGlInit() {
  [[maybe_unused]] static const bool done = []() {
    vtkRenderingOpenGL2_AutoInit_Construct();
    return true;
  }();
}

}  // namespace internal
}  // namespace drake
