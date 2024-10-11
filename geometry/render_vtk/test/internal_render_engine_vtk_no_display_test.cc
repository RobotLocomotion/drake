#include <stdlib.h>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render_vtk/factory.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

// Invoke RenderEngineVtk::RenderColorImage.
void TryToRenderSomething() {
  const RenderEngineVtkParams params;
  std::unique_ptr<render::RenderEngine> dut = MakeRenderEngineVtk(params);
  const int kWidth = 640;
  const int kHeight = 480;
  const double kClipNear = 0.1;
  const double kClipFar = 100.0;
  const double kFovY = M_PI_4;
  const ColorRenderCamera color_camera(
      {"unused", {kWidth, kHeight, kFovY}, {kClipNear, kClipFar}, {}});
  systems::sensors::ImageRgba8U image(kWidth, kHeight);
  dut->RenderColorImage(color_camera, &image);
}

// When DISPLAY is not set, we get a specific message. Additional warnings from
// VTK that might explain the problem should appear as `log()->warn(...)` text,
// but we don't test for that here.
GTEST_TEST(RenderEngineVtkNoDisplayTest, MissingDisplay) {
  // Clear the environment.
  EXPECT_EQ(::unsetenv("DISPLAY"), 0);

  // Rendering should throw.
  DRAKE_EXPECT_THROWS_MESSAGE(TryToRenderSomething(),
                              ".*vtkWindow.*RenderEngineVtk.*");
}

// Ditto when DISPLAY is incorrectly set.
GTEST_TEST(RenderEngineVtkNoDisplayTest, BadDisplay) {
  // Clear the environment.
  EXPECT_EQ(::setenv("DISPLAY", "Hello", 1), 0);

  // Rendering should throw.
  DRAKE_EXPECT_THROWS_MESSAGE(TryToRenderSomething(),
                              ".*vtkWindow.*RenderEngineVtk.*");
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
