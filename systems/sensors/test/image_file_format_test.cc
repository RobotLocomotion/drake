#include "drake/systems/sensors/image_file_format.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace sensors {

// Checks for fmt capability. A spot-check of just one enum value is sufficient.
GTEST_TEST(ImageFileFormatTest, ToString) {
  EXPECT_EQ(fmt::to_string(ImageFileFormat::kPng), "png");
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
