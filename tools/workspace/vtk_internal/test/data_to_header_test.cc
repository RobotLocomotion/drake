#include <string>

#define VTK_ABI_NAMESPACE_BEGIN
#define VTK_ABI_NAMESPACE_END

#include <BlueNoiseTexture64x64.h>  // vtkRenderingOpenGl2
#include <gtest/gtest.h>
#include <vtkDepthOfFieldPassFS.h>

#include "drake/common/find_runfiles.h"

namespace drake {
namespace {

// Confirm that the in-memory C++-representation of the data files match the
// data files.
GTEST_TEST(InternalVtkTest, BinaryFile) {
  // The C++ data is available and defined.
  ASSERT_GT(sizeof(BlueNoiseTexture64x64), 0);

  const std::string path =
      FindRunfile(
          "vtk_internal/Rendering/OpenGL2/textures/BlueNoiseTexture64x64.jpg")
          .abspath;
  ASSERT_FALSE(path.empty());

  // 1. Read the file bytes.
  // 2. Compare those bytes with the bytes from in-memory.
}

GTEST_TEST(InternalVtkTest, AsciiFile) {
  // The C++ data is available and defined.
  ASSERT_GT(sizeof(vtkDepthOfFieldPassFS), 0);

  const std::string path =
      FindRunfile(
          "vtk_internal/Rendering/OpenGL2/glsl/vtkDepthOfFieldPassFS.glsl")
          .abspath;
  ASSERT_FALSE(path.empty());

  // 1. Read file text.
  // 2. Strip off leading and trailing whitespace from both.
  // 3. Compare strings.
}

}  // namespace
}  // namespace drake
