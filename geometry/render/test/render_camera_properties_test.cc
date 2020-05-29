#include "drake/geometry/render/render_camera_properties.h"

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

GTEST_TEST(RenderCameraPropertiesTest, Constructor) {
  {
    // Case: Default clipping values are valid.
    RenderCameraProperties props{"name"};
    EXPECT_EQ(props.render_engine_name(), "name");
    EXPECT_GT(props.near_clipping_plane(), 0);
    EXPECT_GT(props.far_clipping_plane(), 0);
    EXPECT_GT(props.far_clipping_plane(), props.near_clipping_plane());
  }

  {
    // Case: Full constructor validates clipping planes.
    const char* error_message =
        "The clipping plane values must both be positive and the far clipping "
        "plane must be greater than the near. Instantiated with near = {} and "
        "far = {}";
    DRAKE_EXPECT_THROWS_MESSAGE(RenderCameraProperties("name", -0.1, 10),
                                std::runtime_error,
                                fmt::format(error_message, -0.1, 10.));
    DRAKE_EXPECT_THROWS_MESSAGE(RenderCameraProperties("name", 0.1, -10),
                                std::runtime_error,
                                fmt::format(error_message, 0.1, -10.0));
    DRAKE_EXPECT_THROWS_MESSAGE(RenderCameraProperties("name", 1.5, 1.0),
                                std::runtime_error,
                                fmt::format(error_message, 1.5, 1.0));
  }
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
