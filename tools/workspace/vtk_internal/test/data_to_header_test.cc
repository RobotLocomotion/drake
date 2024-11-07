#include <string>

// clang-format off
#include <gtest/gtest.h>

// The converted header files require these macros be defined.
#define VTK_ABI_NAMESPACE_BEGIN
#define VTK_ABI_NAMESPACE_END
#include <BlueNoiseTexture64x64.h>  // vtkRenderingOpenGl2
#include <vtkDepthOfFieldPassFS.h>  // vtkRenderingOpenGl2
// clang-format on

#include "drake/common/find_resource.h"
#include "drake/common/find_runfiles.h"

namespace drake {
namespace {

// Confirm that the in-memory C++-representation of the data files match the
// data files.
GTEST_TEST(InternalVtkTest, BinaryFile) {
  // The C++ data is available and defined.
  ASSERT_GT(sizeof(BlueNoiseTexture64x64), 0);
  const std::string memory_contents(
      std::string_view(reinterpret_cast<const char*>(BlueNoiseTexture64x64),
                       sizeof(BlueNoiseTexture64x64)));
  ASSERT_FALSE(memory_contents.empty());

  const std::string path =
      FindRunfile(
          "vtk_internal/Rendering/OpenGL2/textures/BlueNoiseTexture64x64.jpg")
          .abspath;
  ASSERT_FALSE(path.empty());
  const std::string disk_contents = ReadFileOrThrow(path);

  EXPECT_EQ(memory_contents, disk_contents);
}

GTEST_TEST(InternalVtkTest, AsciiFile) {
  // The C++ data is available and defined.
  ASSERT_GT(sizeof(vtkDepthOfFieldPassFS), 0);
  const std::string memory_contents(vtkDepthOfFieldPassFS);
  ASSERT_FALSE(memory_contents.empty());

  const std::string path =
      FindRunfile(
          "vtk_internal/Rendering/OpenGL2/glsl/vtkDepthOfFieldPassFS.glsl")
          .abspath;
  ASSERT_FALSE(path.empty());
  const std::string disk_contents = ReadFileOrThrow(path);

  // The conversion may embed the ascii with some surrounding whitespace; we
  // don't care. We'll simply trim trailing/leading whitespace and then compare.
  auto trim = [](const std::string& s) {
    auto start = s.find_first_not_of(" \n\t");
    auto last = s.find_last_not_of(" \n\t");
    return std::string_view(s.data() + start, last - start + 1);
  };

  EXPECT_EQ(trim(memory_contents), trim(disk_contents));
}

}  // namespace
}  // namespace drake
