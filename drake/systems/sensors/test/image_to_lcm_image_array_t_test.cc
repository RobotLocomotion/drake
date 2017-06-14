#include "drake/systems/sensors/image_to_lcm_image_array_t.h"

#include <gtest/gtest.h>
#include "robotlocomotion/image_array_t.hpp"

#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {

const char* kColorFrameName = "color_frame_name";
const char* kDepthFrameName = "depth_frame_name";
const char* kLabelFrameName = "label_frame_name";

// Using very tiny image.
const int kImageWidth = 8;
const int kImageHeight = 6;

robotlocomotion::image_array_t SetUpInputAndOutput(
    ImageToLcmImageArrayT* dut, const ImageBgra8U& color_image,
    const ImageDepth32F& depth_image, const ImageLabel16I& label_image) {
  const InputPortDescriptor<double>& color_image_input_port =
      dut->color_image_input_port();
  const InputPortDescriptor<double>& depth_image_input_port =
      dut->depth_image_input_port();
  const InputPortDescriptor<double>& label_image_input_port =
      dut->label_image_input_port();

  auto color_image_value = std::make_unique<Value<ImageBgra8U>>(color_image);
  auto depth_image_value = std::make_unique<Value<ImageDepth32F>>(depth_image);
  auto label_image_value = std::make_unique<Value<ImageLabel16I>>(label_image);

  std::unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  context->FixInputPort(color_image_input_port.get_index(),
                        std::move(color_image_value));
  context->FixInputPort(depth_image_input_port.get_index(),
                        std::move(depth_image_value));
  context->FixInputPort(label_image_input_port.get_index(),
                        std::move(label_image_value));

  auto output = dut->AllocateOutput(*context);
  dut->CalcOutput(*context, output.get());

  auto output_image_array_t = output->get_data(
      dut->image_array_t_msg_output_port().get_index())->GetValue<
        robotlocomotion::image_array_t>();

  return output_image_array_t;
}


GTEST_TEST(ImageToLcmImageArrayT, ValidTest) {
  ImageBgra8U color_image(kImageWidth, kImageHeight);
  ImageDepth32F depth_image(kImageWidth, kImageHeight);
  ImageLabel16I label_image(kImageWidth, kImageHeight);

  ImageToLcmImageArrayT dut(kColorFrameName, kDepthFrameName, kLabelFrameName);

  auto output_image_array_t =
      SetUpInputAndOutput(&dut, color_image, depth_image, label_image);

  // Verifyies image_array_t
  EXPECT_EQ(output_image_array_t.header.seq, 0);
  EXPECT_EQ(output_image_array_t.header.utime, 0);
  EXPECT_EQ(output_image_array_t.header.frame_name, "");
  EXPECT_EQ(output_image_array_t.num_images, 3);
  EXPECT_EQ(output_image_array_t.images.size(), 3);

  // Verifyies each image_t.
  for (int i = 0; i < output_image_array_t.num_images; ++i) {
    auto const& image = output_image_array_t.images[i];

    EXPECT_EQ(image.header.seq, 0);
    EXPECT_EQ(image.header.utime, 0);
    EXPECT_EQ(image.width, kImageWidth);
    EXPECT_EQ(image.height, kImageHeight);
    EXPECT_EQ(image.data.size(), image.size);
    EXPECT_FALSE(image.bigendian);
    // This is a workaround for const variable to avoid undefined reference
    // error: https://goo.gl/3PU0eq
    uint8_t expected_compression_method =
        robotlocomotion::image_t::COMPRESSION_METHOD_ZLIB;
    EXPECT_EQ(image.compression_method, expected_compression_method);

    std::string frame_name;
    int row_stride;
    int8_t pixel_format;
    int8_t channel_type;
    switch (i) {
      case 0: {
        frame_name = kColorFrameName;
        row_stride = color_image.width() * color_image.kNumChannels *
            sizeof(*color_image.at(0, 0));
        pixel_format = robotlocomotion::image_t::PIXEL_FORMAT_BGRA;
        channel_type = robotlocomotion::image_t::CHANNEL_TYPE_UINT8;
        break;
      }
      case 1: {
        frame_name = kDepthFrameName;
        row_stride = depth_image.width() * depth_image.kNumChannels *
            sizeof(*depth_image.at(0, 0));
        pixel_format = robotlocomotion::image_t::PIXEL_FORMAT_DEPTH;
        channel_type = robotlocomotion::image_t::CHANNEL_TYPE_FLOAT32;
        break;
      }
      case 2: {
        frame_name = kLabelFrameName;
        row_stride = label_image.width() * label_image.kNumChannels *
            sizeof(*label_image.at(0, 0));
        pixel_format = robotlocomotion::image_t::PIXEL_FORMAT_LABEL;
        channel_type = robotlocomotion::image_t::CHANNEL_TYPE_INT16;
        break;
      }
      default: {
        EXPECT_FALSE(true);
        break;
      }
    }

    EXPECT_EQ(image.header.frame_name, frame_name);
    EXPECT_EQ(image.row_stride, row_stride);
    EXPECT_EQ(image.pixel_format, pixel_format);
    EXPECT_EQ(image.channel_type, channel_type);
  }
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
