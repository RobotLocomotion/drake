#include "drake/geometry/render/colorize_image.h"

#include <gtest/gtest.h>

#include "drake/geometry/render/render_label.h"
#include "drake/systems/sensors/test_utilities/image_compare.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

// Runs all of the code once and spot checks a sample image.
GTEST_TEST(ColorizeLabelImageTest, Basic) {
  // A label image with just one non-empty value. Note that kEmpty is NOT a
  // literal zero.
  ImageLabel16I label(6, 2, static_cast<int>(RenderLabel::kEmpty));
  label.at(5, 1)[0] = 0;

  // Expect a color image with a matching value. Label 0 happens to be "tableau
  // blue" (#1F77B4). The default background color is transparent black.
  ImageRgba8U expected(6, 2, 0);
  expected.at(5, 1)[0] = 0x1F;
  expected.at(5, 1)[1] = 0x77;
  expected.at(5, 1)[2] = 0xB4;
  expected.at(5, 1)[3] = 0xFF;

  // Check the colorized image; note that `actual` starts out empty, so this
  // also confirms that the output gets resized to match the input.
  ImageRgba8U actual;
  ColorizeLabelImage(label, &actual);
  EXPECT_EQ(actual, expected);
}

// Checks colorization with a non-default background color.
GTEST_TEST(ColorizeLabelImageTest, Background) {
  // A label image with just one non-empty value.
  ImageLabel16I label(6, 2, static_cast<int>(RenderLabel::kEmpty));
  label.at(5, 1)[0] = 0;

  ImageRgba8U actual;
  ColorizeLabelImage(label, &actual, Rgba(0.2, 0.2, 0.2, 1.0));

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
}  // namespace render
}  // namespace geometry
}  // namespace drake
