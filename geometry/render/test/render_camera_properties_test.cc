#include "drake/geometry/render/render_camera_properties.h"

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

GTEST_TEST(DepthRangeTest, Constructor) {
  {
    // Case: valid range values.
    DepthRange range{0.1, 11.5};
    EXPECT_EQ(range.min_depth(), 0.1);
    EXPECT_EQ(range.max_depth(), 11.5);
  }

  {
    // Case: Bad range values throw.
    const char* error_message =
        "The depth range values must both be positive and the maximum depth "
        "must be greater than the minimum depth. Instantiated with min = {} "
        "and max = {}";
    DRAKE_EXPECT_THROWS_MESSAGE(DepthRange(-0.1, 10),
                                std::runtime_error,
                                fmt::format(error_message, -0.1, 10.));
    DRAKE_EXPECT_THROWS_MESSAGE(DepthRange(0.1, -10),
                                std::runtime_error,
                                fmt::format(error_message, 0.1, -10.0));
    DRAKE_EXPECT_THROWS_MESSAGE(DepthRange(1.5, 1.0),
                                std::runtime_error,
                                fmt::format(error_message, 1.5, 1.0));
  }
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
