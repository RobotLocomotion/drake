#include "drake/systems/sensors/image_to_lcm_image_array_t.h"

#include <gtest/gtest.h>

#include "drake/lcmt_image_array.hpp"
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

lcmt_image_array SetUpInputAndOutput(
    ImageToLcmImageArrayT* dut, const ImageRgba8U& color_image,
    const ImageDepth32F& depth_image, const ImageLabel16I& label_image) {
  const InputPort<double>& color_image_input_port =
      dut->color_image_input_port();
  const InputPort<double>& depth_image_input_port =
      dut->depth_image_input_port();
  const InputPort<double>& label_image_input_port =
      dut->label_image_input_port();

  std::unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  color_image_input_port.FixValue(context.get(), color_image);
  depth_image_input_port.FixValue(context.get(), depth_image);
  label_image_input_port.FixValue(context.get(), label_image);

  return dut->image_array_t_msg_output_port().
      Eval<lcmt_image_array>(*context);
}

GTEST_TEST(ImageToLcmImageArrayT, ValidTest) {
  ImageRgba8U color_image(kImageWidth, kImageHeight);
  ImageDepth32F depth_image(kImageWidth, kImageHeight);
  ImageLabel16I label_image(kImageWidth, kImageHeight);

  auto Verify = [color_image, depth_image, label_image](
                   const ImageToLcmImageArrayT& dut,
                   const lcmt_image_array& output_image_array,
                   uint8_t compression_method) {
    // Verifyies lcmt_image_array
    EXPECT_EQ(output_image_array.header.seq, 0);
    EXPECT_EQ(output_image_array.header.utime, 0);
    EXPECT_EQ(output_image_array.header.frame_name, "");
    EXPECT_EQ(output_image_array.num_images, 3);
    EXPECT_EQ(output_image_array.images.size(), 3);

    // Verifyies each lcmt_image.
    for (auto const& image : output_image_array.images) {
      EXPECT_EQ(image.header.seq, 0);
      EXPECT_EQ(image.header.utime, 0);
      EXPECT_EQ(image.width, kImageWidth);
      EXPECT_EQ(image.height, kImageHeight);
      EXPECT_EQ(image.data.size(), image.size);
      EXPECT_FALSE(image.bigendian);
      EXPECT_EQ(image.compression_method, compression_method);

      std::string frame_name;
      int row_stride;
      int8_t pixel_format;
      int8_t channel_type;
      if (image.pixel_format == lcmt_image::PIXEL_FORMAT_RGBA) {
        frame_name = kColorFrameName;
        row_stride = color_image.width() * color_image.kNumChannels *
            sizeof(*color_image.at(0, 0));
        pixel_format = lcmt_image::PIXEL_FORMAT_RGBA;
        channel_type = lcmt_image::CHANNEL_TYPE_UINT8;
      } else if (image.pixel_format == lcmt_image::PIXEL_FORMAT_DEPTH) {
        frame_name = kDepthFrameName;
        row_stride = depth_image.width() * depth_image.kNumChannels *
            sizeof(*depth_image.at(0, 0));
        pixel_format = lcmt_image::PIXEL_FORMAT_DEPTH;
        channel_type = lcmt_image::CHANNEL_TYPE_FLOAT32;
      } else if (image.pixel_format == lcmt_image::PIXEL_FORMAT_LABEL) {
        frame_name = kLabelFrameName;
        row_stride = label_image.width() * label_image.kNumChannels *
            sizeof(*label_image.at(0, 0));
        pixel_format = lcmt_image::PIXEL_FORMAT_LABEL;
        channel_type = lcmt_image::CHANNEL_TYPE_INT16;
      } else {
        EXPECT_FALSE(true);
      }

      EXPECT_EQ(image.header.frame_name, frame_name);
      EXPECT_EQ(image.row_stride, row_stride);
      EXPECT_EQ(image.pixel_format, pixel_format);
      EXPECT_EQ(image.channel_type, channel_type);
    }
  };

  ImageToLcmImageArrayT dut_compressed(
      kColorFrameName, kDepthFrameName, kLabelFrameName, true);
  auto image_array_t_compressed = SetUpInputAndOutput(
          &dut_compressed, color_image, depth_image, label_image);
  Verify(dut_compressed, image_array_t_compressed,
         lcmt_image::COMPRESSION_METHOD_ZLIB);

  ImageToLcmImageArrayT dut_uncompressed(
      kColorFrameName, kDepthFrameName, kLabelFrameName, false);
  auto image_array_t_uncompressed = SetUpInputAndOutput(
      &dut_uncompressed, color_image, depth_image, label_image);
  Verify(dut_uncompressed, image_array_t_uncompressed,
         lcmt_image::COMPRESSION_METHOD_NOT_COMPRESSED);
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
