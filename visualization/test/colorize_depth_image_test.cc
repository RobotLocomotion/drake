#include "drake/visualization/colorize_depth_image.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/sensors/test_utilities/image_compare.h"

namespace drake {
namespace visualization {
namespace {

using geometry::Rgba;
using systems::sensors::ImageDepth16U;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageRgba8U;
using systems::sensors::ImageTraits;
using systems::sensors::PixelType;

/* Creates a 32F depth image with only two valid values. */
ImageDepth32F MakeSample32F() {
  ImageDepth32F result(6, 2, std::numeric_limits<float>::infinity());
  result.at(0, 0)[0] = -3.0;
  result.at(5, 0)[0] = 1.0;
  result.at(5, 1)[0] = 2.0;
  return result;
}

/* Creates a 16U depth image with only two valid values. */
ImageDepth16U MakeSample16U() {
  ImageDepth16U result(6, 2, 0);
  result.at(0, 0)[0] = ImageTraits<PixelType::kDepth16U>::kTooFar;
  result.at(5, 0)[0] = 1000;
  result.at(5, 1)[0] = 2000;
  return result;
}

/* Returns the "invalid depth" placeholder color. */
Rgba MakeInvalidColor() {
  return Rgba(0.2, 0.0, 0.0, 1.0);
}

/* Creates a color image with the expected values, i.e., the closest pixel is
white and the farthest pixel is black. The rest of the pixels are set to the
default invalid color. */
ImageRgba8U MakeExpected() {
  ImageRgba8U result(6, 2);
  for (int u = 0; u < result.width(); ++u) {
    for (int v = 0; v < result.height(); ++v) {
      if (u < 5) {
        // Per MakeInvalidColor() the invalid_color is (51, 0, 0).
        result.at(u, v)[0] = 51;
      } else {
        for (int ch = 0; ch < 3; ++ch) {
          result.at(u, v)[ch] = (v == 0) ? 255 : 0;
        }
      }
      result.at(u, v)[3] = 255;
    }
  }
  return result;
}

// Runs all of the code once and spot checks a sample 32F image.
GTEST_TEST(ColorDepthImageTest, Basic32F) {
  ColorizeDepthImage<double> dut;
  dut.set_invalid_color(MakeInvalidColor());
  auto context = dut.CreateDefaultContext();
  dut.GetInputPort("depth_image_32f").FixValue(context.get(), MakeSample32F());
  const auto& actual =
      dut.GetOutputPort("color_image").template Eval<ImageRgba8U>(*context);
  EXPECT_EQ(actual, MakeExpected());
}

// Runs all of the code once and spot checks a sample 16U image.
GTEST_TEST(ColorDepthImageTest, Basic16U) {
  ColorizeDepthImage<double> dut;
  dut.set_invalid_color(MakeInvalidColor());
  auto context = dut.CreateDefaultContext();
  dut.GetInputPort("depth_image_16u").FixValue(context.get(), MakeSample16U());
  const auto& actual =
      dut.GetOutputPort("color_image").template Eval<ImageRgba8U>(*context);
  EXPECT_EQ(actual, MakeExpected());
}

// Checks the direct Calc() function for a 32F input.
GTEST_TEST(ColorDepthImageTest, Direct32F) {
  ColorizeDepthImage<double> dut;
  dut.set_invalid_color(MakeInvalidColor());
  ImageRgba8U actual;
  dut.Calc(MakeSample32F(), &actual);
  EXPECT_EQ(actual, MakeExpected());
}

// Checks the direct Calc() function for a 16U input.
GTEST_TEST(ColorDepthImageTest, Direct16U) {
  ColorizeDepthImage<double> dut;
  dut.set_invalid_color(MakeInvalidColor());
  ImageRgba8U actual;
  dut.Calc(MakeSample16U(), &actual);
  EXPECT_EQ(actual, MakeExpected());
}

// Confirm the default value.
GTEST_TEST(ColorDepthImageTest, DefaultInvalidColor) {
  ColorizeDepthImage<double> dut;
  const Rgba invalid = dut.get_invalid_color();
  EXPECT_EQ(static_cast<int>(invalid.r() * 255), 100);
  EXPECT_EQ(static_cast<int>(invalid.g() * 255), 0);
  EXPECT_EQ(static_cast<int>(invalid.b() * 255), 0);
  EXPECT_EQ(static_cast<int>(invalid.a() * 255), 255);
}

// Colorizes an image with valid, uniform depth across the board. The image
// should be all white. This requires that the implementation take care when
// scaling the depth range not to divide by zero.
GTEST_TEST(ColorDepthImageTest, UniformDepth) {
  const ImageDepth32F input(6, 2, 22.0f);
  const ImageRgba8U expected(6, 2, uint8_t{255});
  ColorizeDepthImage<double> dut;
  ImageRgba8U actual;
  dut.Calc(input, &actual);
  EXPECT_EQ(actual, expected);
}

GTEST_TEST(ColorDepthImageTest, BadlyConnectedInputs) {
  ColorizeDepthImage<double> dut;
  auto context = dut.CreateDefaultContext();

  // No values for both input ports should raise an exception. This is not a
  // great error message, but at least we have a hint as to what failed.
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.GetOutputPort("color_image").template Eval<ImageRgba8U>(*context),
      ".*has_depth32f != has_depth16u.*");

  // Setting values for both input ports should also raise an exception.
  dut.GetInputPort("depth_image_32f").FixValue(context.get(), MakeSample32F());
  dut.GetInputPort("depth_image_16u").FixValue(context.get(), MakeSample16U());
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.GetOutputPort("color_image").template Eval<ImageRgba8U>(*context),
      // Likewise not a great error message.
      ".*has_depth32f != has_depth16u.*");
}

}  // namespace
}  // namespace visualization
}  // namespace drake
