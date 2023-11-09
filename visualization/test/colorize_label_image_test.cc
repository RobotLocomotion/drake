#include "drake/visualization/colorize_label_image.h"

#include <gtest/gtest.h>

#include "drake/geometry/render/render_label.h"
#include "drake/systems/sensors/test_utilities/image_compare.h"

namespace drake {
namespace visualization {
namespace {

using geometry::Rgba;
using geometry::render::RenderLabel;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

// Runs all of the code once and spot checks a sample image.
GTEST_TEST(ColorLabelImageTest, Basic) {
  ColorizeLabelImage<double> dut;
  auto context = dut.CreateDefaultContext();

  // Input a label image with just one non-empty value. Note that kEmpty is NOT
  // a literal zero.
  ImageLabel16I label(6, 2, static_cast<int>(RenderLabel::kEmpty));
  label.at(5, 1)[0] = 0;
  dut.GetInputPort("label_image").FixValue(context.get(), label);

  // Expect a color image with a matching value. Label 0 happens to be "tableau
  // blue" (#1F77B4).
  ImageRgba8U expected(6, 2, 0);
  expected.at(5, 1)[0] = 0x1F;
  expected.at(5, 1)[1] = 0x77;
  expected.at(5, 1)[2] = 0xB4;
  expected.at(5, 1)[3] = 0xFF;

  // Check the colorized image.
  const auto& actual =
      dut.GetOutputPort("color_image").template Eval<ImageRgba8U>(*context);
  EXPECT_EQ(actual, expected);
}

// Checks the Calc() function when run with a non-default background color.
GTEST_TEST(ColorLabelImageTest, DirectCalcWithBackground) {
  ColorizeLabelImage<double> dut;

  // Make a label image with just one non-empty value.
  ImageLabel16I label(6, 2, static_cast<int>(RenderLabel::kEmpty));
  label.at(5, 1)[0] = 0;

  // Choose a non-default background color.
  dut.set_background_color(Rgba(0.2, 0.2, 0.2, 1.0));
  EXPECT_EQ(dut.get_background_color(), Rgba(0.2, 0.2, 0.2, 1.0));

  // Colorize.
  ImageRgba8U actual;
  dut.Calc(label, &actual);

  // Check an arbitrary empty pixel.
  EXPECT_EQ(actual.at(0, 0)[0], 51);
  EXPECT_EQ(actual.at(0, 0)[1], 51);
  EXPECT_EQ(actual.at(0, 0)[2], 51);
  EXPECT_EQ(actual.at(0, 0)[3], 255);

  // Check the non-empty pixel.
  EXPECT_EQ(actual.at(5, 1)[0], 0x1F);
  EXPECT_EQ(actual.at(5, 1)[1], 0x77);
  EXPECT_EQ(actual.at(5, 1)[2], 0xB4);
  EXPECT_EQ(actual.at(5, 1)[3], 0xFF);
}

}  // namespace
}  // namespace visualization
}  // namespace drake
