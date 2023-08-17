#include "drake/systems/sensors/pixel_types.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace sensors {
namespace {

GTEST_TEST(PixelTypesTest, Formatters) {
  EXPECT_EQ(fmt::to_string(PixelType::kRgba8U), "Rgba8U");
  EXPECT_EQ(fmt::to_string(PixelFormat::kRgba), "Rgba");
  EXPECT_EQ(fmt::to_string(PixelScalar::k8U), "8U");
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
