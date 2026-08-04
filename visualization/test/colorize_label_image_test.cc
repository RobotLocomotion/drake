#include "drake/visualization/colorize_label_image.h"

#include <gtest/gtest.h>

#include "drake/geometry/render/colorize_image.h"
#include "drake/geometry/render/render_label.h"
#include "drake/systems/sensors/test_utilities/image_compare.h"

namespace drake {
namespace visualization {
namespace {

using geometry::Rgba;
using geometry::render::RenderLabel;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

// The colorization itself is tested in geometry/render; here we merely confirm
// that the System is wired up to that function correctly.

GTEST_TEST(ColorizeLabelImageTest, Ports) {
  const ColorizeLabelImage<double> dut;
  ASSERT_EQ(dut.num_input_ports(), 1);
  ASSERT_EQ(dut.num_output_ports(), 1);
  EXPECT_EQ(dut.GetInputPort("label_image").get_index(), 0);
  EXPECT_EQ(dut.GetOutputPort("color_image").get_index(), 0);
}

GTEST_TEST(ColorizeLabelImageTest, BackgroundColorProperty) {
  ColorizeLabelImage<double> dut;
  EXPECT_EQ(dut.get_background_color(), Rgba(0, 0, 0, 0));
  dut.set_background_color(Rgba(0.2, 0.2, 0.2, 1.0));
  EXPECT_EQ(dut.get_background_color(), Rgba(0.2, 0.2, 0.2, 1.0));
}

// Both the output port and the Calc() shortcut must colorize the input using
// the background color property.
GTEST_TEST(ColorizeLabelImageTest, Delegation) {
  ColorizeLabelImage<double> dut;
  dut.set_background_color(Rgba(0.2, 0.2, 0.2, 1.0));

  // A label image with just one non-empty value. Note that kEmpty is NOT a
  // literal zero.
  ImageLabel16I label(6, 2, static_cast<int>(RenderLabel::kEmpty));
  label.at(5, 1)[0] = 0;

  ImageRgba8U expected;
  geometry::render::ColorizeLabelImage(label, &expected,
                                       dut.get_background_color());

  auto context = dut.CreateDefaultContext();
  dut.GetInputPort("label_image").FixValue(context.get(), label);
  EXPECT_EQ(dut.GetOutputPort("color_image").Eval<ImageRgba8U>(*context),
            expected);

  ImageRgba8U actual;
  dut.Calc(label, &actual);
  EXPECT_EQ(actual, expected);
}

}  // namespace
}  // namespace visualization
}  // namespace drake
