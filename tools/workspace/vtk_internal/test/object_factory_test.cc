#include <gtest/gtest.h>
#include <vtkCameraPass.h>      // vtkRenderingCore
#include <vtkClearRGBPass.h>    // vtkRenderingCore
#include <vtkNew.h>             // vtkCommonCore
#include <vtkPolyDataMapper.h>  // vtkRenderingCore
#include <vtkRenderer.h>        // vtkRenderingCore

namespace drake {
namespace {

// Confirms that the vtkRenderingOpenGL2ObjectFactory registers the OpenGL
// implementations of VTK's abstract rendering classes. When an override is
// missing, VTK silently instantiates the (possibly no-op) base class.
GTEST_TEST(InternalVtkTest, ObjectFactoryOverrides) {
  EXPECT_TRUE(vtkNew<vtkRenderer>()->IsA("vtkOpenGLRenderer"));
  EXPECT_TRUE(vtkNew<vtkPolyDataMapper>()->IsA("vtkOpenGLPolyDataMapper"));
  EXPECT_TRUE(vtkNew<vtkCameraPass>()->IsA("vtkOpenGLCameraPass"));
  EXPECT_TRUE(vtkNew<vtkClearRGBPass>()->IsA("vtkOpenGLClearRGBPass"));
}

}  // namespace
}  // namespace drake
