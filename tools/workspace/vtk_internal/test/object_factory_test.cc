#include <gtest/gtest.h>
#include <vtkCameraPass.h>      // vtkRenderingCore
#include <vtkClearRGBPass.h>    // vtkRenderingCore
#include <vtkNew.h>             // vtkCommonCore
#include <vtkPolyDataMapper.h>  // vtkRenderingCore
#include <vtkRenderer.h>        // vtkRenderingCore

#include "drake/tools/workspace/vtk_internal/vtk_opengl_init.h"

namespace drake {
namespace {

// Confirms that the vtkRenderingOpenGL2ObjectFactory registers the OpenGL
// implementations of VTK's abstract rendering classes. When an override is
// missing, VTK silently instantiates the (possibly no-op) base class.
//
// This is all one test case because the factory registration is process-wide
// state, so the "before" checks only make sense prior to the first init call.
GTEST_TEST(InternalVtkTest, ObjectFactoryOverrides) {
  // Merely linking the library does not register the factory (i.e., there is
  // no global constructor doing it behind our back).
  EXPECT_FALSE(vtkNew<vtkRenderer>()->IsA("vtkOpenGLRenderer"));

  internal::VtkOpenGlInit();
  EXPECT_TRUE(vtkNew<vtkRenderer>()->IsA("vtkOpenGLRenderer"));
  EXPECT_TRUE(vtkNew<vtkPolyDataMapper>()->IsA("vtkOpenGLPolyDataMapper"));
  EXPECT_TRUE(vtkNew<vtkCameraPass>()->IsA("vtkOpenGLCameraPass"));
  EXPECT_TRUE(vtkNew<vtkClearRGBPass>()->IsA("vtkOpenGLClearRGBPass"));

  // Calling it again is harmless.
  internal::VtkOpenGlInit();
  EXPECT_TRUE(vtkNew<vtkRenderer>()->IsA("vtkOpenGLRenderer"));
}

}  // namespace
}  // namespace drake
