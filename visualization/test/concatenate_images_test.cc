#include "drake/visualization/concatenate_images.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/systems/sensors/test_utilities/image_compare.h"

namespace {
struct WH {
  int width{};
  int height{};
};
std::string to_string(const WH& wh) {
  return fmt::format("{}x{}", wh.width, wh.height);
}
}  // namespace
DRAKE_FORMATTER_AS(, , WH, x, to_string(x));

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

// Checks the output width and height for various input image sizes.
GTEST_TEST(ConcatenateImagesTest, TilingSize) {
  // Prepare the suite of tests.
  using InputSizes = MatrixX<WH>;
  struct TestCase {
    InputSizes input_sizes;
    WH expected_output_size;
  };
  // clang-format off
  std::vector<TestCase> test_cases{
      // All input images are zero size.
      TestCase{.input_sizes = (InputSizes(1, 1) <<
                   WH{0, 0}
               ).finished(), .expected_output_size = {0, 0}},  // NOLINT
      TestCase{.input_sizes = (InputSizes(3, 2) <<
                   WH{0, 0}, WH{0, 0}, WH{0, 0},
                   WH{0, 0}, WH{0, 0}, WH{0, 0}
               ).finished(), .expected_output_size = {0, 0}},  // NOLINT
      // All input images are a uniform non-zero size.
      TestCase{.input_sizes = (InputSizes(1, 1) <<
                   WH{4, 2}
               ).finished(), .expected_output_size = {4, 2}},  // NOLINT
      TestCase{.input_sizes = (InputSizes(1, 3) <<
                   WH{4, 2}, WH{4, 2}, WH{4, 2}
               ).finished(), .expected_output_size = {12, 2}},  // NOLINT
      TestCase{.input_sizes = (InputSizes(3, 1) <<
                   WH{4, 2},
                   WH{4, 2},
                   WH{4, 2}
               ).finished(), .expected_output_size = {4, 6}},  // NOLINT
      TestCase{.input_sizes = (InputSizes(3, 2) <<
                   WH{4, 2}, WH{4, 2},
                   WH{4, 2}, WH{4, 2},
                   WH{4, 2}, WH{4, 2}
               ).finished(), .expected_output_size = {8, 6}},  // NOLINT
      // Some input images are empty; the larger images still dictate the size.
      TestCase{.input_sizes = (InputSizes(3, 2) <<
                   WH{4, 2}, WH{0, 0},
                   WH{4, 2}, WH{0, 0},
                   WH{0, 0}, WH{4, 2}
               ).finished(), .expected_output_size = {8, 6}},  // NOLINT
      // When a whole row is missing, it does not claim any space in the output.
      TestCase{.input_sizes = (InputSizes(3, 2) <<
                   WH{4, 2}, WH{0, 0},
                   WH{0, 0}, WH{0, 0},
                   WH{0, 0}, WH{4, 2}
               ).finished(), .expected_output_size = {8, 4}},  // NOLINT
      // When a whole col is missing, it does not claim any space in the output.
      TestCase{.input_sizes = (InputSizes(3, 2) <<
                   WH{4, 2}, WH{0, 0},
                   WH{4, 2}, WH{0, 0},
                   WH{4, 2}, WH{0, 0}
               ).finished(), .expected_output_size = {4, 6}},  // NOLINT
  };
  // clang-format on

  // Run the test cases one at a time.
  for (const TestCase& test : test_cases) {
    SCOPED_TRACE(fmt::format("input_sizes =\n{}", fmt_eigen(test.input_sizes)));
    const int rows = test.input_sizes.rows();
    const int cols = test.input_sizes.cols();
    const ConcatenateImages<double> dut(rows, cols);
    auto context = dut.CreateDefaultContext();
    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < cols; ++col) {
        const WH& wh = test.input_sizes(row, col);
        const ImageRgba8U image(wh.width, wh.height);
        dut.get_input_port(row, col).FixValue(context.get(), image);
      }
    }
    const auto& output =
        dut.GetOutputPort("color_image").template Eval<ImageRgba8U>(*context);
    EXPECT_EQ(output.width(), test.expected_output_size.width);
    EXPECT_EQ(output.height(), test.expected_output_size.height);
  }
}

// Spot-checks the actual rgb data during non-uniform tiling.
GTEST_TEST(ConcatenateImagesTest, TilingData) {
  // We use three distinct input images:
  //
  //   red: 1w x 1h image filled with all-red pixels
  // green: 2w x 1h image filled with all-green pixels
  //  blue: 1w x 2h image filled with all-blue pixels
  //
  // We'll tile them into 2 rows and 3 cols as follows:
  //
  //  red  green blue
  //  blue red   green
  //
  // Thus, the output image should be as follows (using 'z' for zero padding):
  //
  //  R G G B z
  //  z z z B z
  //  B R z G G
  //  B z z z z

  // Prepare the inputs.
  auto set_to_red = [](uint8_t* pixel) {
    pixel[0] = 30;
    pixel[3] = 255;
  };
  auto set_to_green = [](uint8_t* pixel) {
    pixel[1] = 60;
    pixel[3] = 255;
  };
  auto set_to_blue = [](uint8_t* pixel) {
    pixel[2] = 90;
    pixel[3] = 255;
  };
  ImageRgba8U red(1, 1);
  set_to_red(red.at(0, 0));
  ImageRgba8U green(2, 1);
  set_to_green(green.at(0, 0));
  set_to_green(green.at(1, 0));
  ImageRgba8U blue(1, 2);
  set_to_blue(blue.at(0, 0));
  set_to_blue(blue.at(0, 1));

  // Prepare the expected output:
  ImageRgba8U expected(5, 4);
  set_to_red(expected.at(0, 0));
  set_to_red(expected.at(1, 2));
  set_to_green(expected.at(1, 0));
  set_to_green(expected.at(2, 0));
  set_to_green(expected.at(3, 2));
  set_to_green(expected.at(4, 2));
  set_to_blue(expected.at(3, 0));
  set_to_blue(expected.at(3, 1));
  set_to_blue(expected.at(0, 2));
  set_to_blue(expected.at(0, 3));

  // Compute the actual output.
  const ConcatenateImages<double> dut(2, 3);
  auto context = dut.CreateDefaultContext();
  dut.get_input_port(0, 0).FixValue(context.get(), red);
  dut.get_input_port(0, 1).FixValue(context.get(), green);
  dut.get_input_port(0, 2).FixValue(context.get(), blue);
  dut.get_input_port(1, 0).FixValue(context.get(), blue);
  dut.get_input_port(1, 1).FixValue(context.get(), red);
  dut.get_input_port(1, 2).FixValue(context.get(), green);
  const auto& actual =
      dut.GetOutputPort("color_image").template Eval<ImageRgba8U>(*context);
  EXPECT_EQ(actual, expected);
}

}  // namespace
}  // namespace visualization
}  // namespace drake
