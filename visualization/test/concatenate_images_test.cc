#include "drake/visualization/concatenate_images.h"

#include <gtest/gtest.h>

#include "drake/systems/sensors/test_utilities/image_compare.h"

namespace drake {
namespace visualization {
namespace {

using systems::sensors::ImageRgba8U;

// Runs all of the code once and spot checks a sample image.
GTEST_TEST(ConcatenateImagesTest, Basic) {
  constexpr int rows = 2;
  constexpr int cols = 3;
  const ConcatenateImages<double> dut(rows, cols);
  auto context = dut.CreateDefaultContext();

  // Set up the inputs.
  int red_counter = 1;
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      ImageRgba8U image(4, 2);
      image.at(0, 0)[0] = red_counter;
      image.at(0, 0)[1] = 0;
      image.at(0, 0)[2] = 0;
      image.at(0, 0)[3] = 255;
      dut.get_input_port(row, col).FixValue(context.get(), image);
      red_counter += 1;
    }
  }

  // Expect the red pixels in the correct order.
  ImageRgba8U expected(12, 4, 0);
  expected.at(0, 0)[0] = 1;
  expected.at(4, 0)[0] = 2;
  expected.at(8, 0)[0] = 3;
  expected.at(0, 2)[0] = 4;
  expected.at(4, 2)[0] = 5;
  expected.at(8, 2)[0] = 6;
  expected.at(0, 0)[3] = 255;
  expected.at(4, 0)[3] = 255;
  expected.at(8, 0)[3] = 255;
  expected.at(0, 2)[3] = 255;
  expected.at(4, 2)[3] = 255;
  expected.at(8, 2)[3] = 255;
  const auto& actual =
      dut.GetOutputPort("color_image").template Eval<ImageRgba8U>(*context);
  EXPECT_EQ(actual, expected);
}

}  // namespace
}  // namespace visualization
}  // namespace drake
