#include "drake/systems/sensors/lcm_image_array_to_images.h"

#include <fstream>
#include <memory>

#include <gtest/gtest.h>
#include "robotlocomotion/image_array_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/systems/sensors/image.h"

using robotlocomotion::image_array_t;
using robotlocomotion::image_t;

namespace drake {
namespace systems {
namespace sensors {
namespace {

void LoadImageData(const std::string& filename, image_t* image) {
  std::ifstream is(filename, std::ios::binary);
  EXPECT_TRUE(is.good());

  image->data.clear();
  while (true) {
    uint8_t c = is.get();
    if (is.eof()) { break; }
    image->data.push_back(c);
  }

  EXPECT_TRUE(is.eof());
  EXPECT_FALSE(is.bad());
  image->size = image->data.size();
}

void DecodeImageArray(
    LcmImageArrayToImages* dut, const image_array_t& lcm_images,
    ImageRgba8U* color_image, ImageDepth32F* depth_image) {
  std::unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  auto input_value = std::make_unique<Value<image_array_t>>(lcm_images);
  context->FixInputPort(dut->image_array_t_input_port().get_index(),
                        std::move(input_value));

  *color_image = dut->color_image_output_port().Eval<ImageRgba8U>(*context);
  *depth_image = dut->depth_image_output_port().Eval<ImageDepth32F>(*context);
}

GTEST_TEST(LcmImageArrayToImagesTest, EmptyArrayTest) {
  image_array_t lcm_images{};

  // Populate our expected images with some width/height and expect they're
  // cleared.
  ImageRgba8U color_image(32, 32);
  ImageDepth32F depth_image(32, 32);
  LcmImageArrayToImages dut;

  DecodeImageArray(&dut, lcm_images, &color_image, &depth_image);
  EXPECT_EQ(color_image.size(), 0);
  EXPECT_EQ(depth_image.size(), 0);
}

GTEST_TEST(LcmImageArrayToImagesTest, JpegTest) {
  // Start with an empty color image and expect that it will be populated.
  ImageRgba8U color_image;

  // Start with a populated depth image, expect it will be cleared.
  ImageDepth32F depth_image(32, 32);

  image_t jpeg_image{};
  jpeg_image.width = 32;
  jpeg_image.height = 32;
  jpeg_image.row_stride = jpeg_image.width * 3;
  jpeg_image.bigendian = 0;
  jpeg_image.pixel_format = image_t::PIXEL_FORMAT_RGB;
  jpeg_image.channel_type = image_t::CHANNEL_TYPE_UINT8;
  jpeg_image.compression_method = image_t::COMPRESSION_METHOD_JPEG;
  LoadImageData(
      FindResourceOrThrow("drake/systems/sensors/test/jpeg_test.jpg"),
      &jpeg_image);

  image_array_t lcm_images{};
  lcm_images.num_images = 1;
  lcm_images.images.push_back(jpeg_image);

  LcmImageArrayToImages dut;
  DecodeImageArray(&dut, lcm_images, &color_image, &depth_image);
  EXPECT_EQ(color_image.size(), 32 * 32 * 4);
  EXPECT_EQ(depth_image.size(), 0);
}

GTEST_TEST(LcmImageArrayToImagesTest, PngTest) {
  image_t png_image{};
  png_image.width = 32;
  png_image.height = 32;
  png_image.row_stride = png_image.width * 3;
  png_image.bigendian = 0;
  png_image.pixel_format = image_t::PIXEL_FORMAT_RGB;
  png_image.channel_type = image_t::CHANNEL_TYPE_UINT8;
  png_image.compression_method = image_t::COMPRESSION_METHOD_PNG;
  LoadImageData(
      FindResourceOrThrow("drake/systems/sensors/test/png_color_test.png"),
      &png_image);

  image_array_t lcm_images{};
  lcm_images.num_images = 1;
  lcm_images.images.push_back(png_image);

  png_image.row_stride = png_image.width * 2;
  png_image.pixel_format = image_t::PIXEL_FORMAT_DEPTH;
  png_image.channel_type = image_t::CHANNEL_TYPE_UINT16;
  LoadImageData(
      FindResourceOrThrow("drake/systems/sensors/test/png_gray16_test.png"),
      &png_image);

  lcm_images.num_images++;
  lcm_images.images.push_back(png_image);

  LcmImageArrayToImages dut;
  ImageRgba8U color_image;
  ImageDepth32F depth_image;

  DecodeImageArray(&dut, lcm_images, &color_image, &depth_image);
  EXPECT_EQ(color_image.size(), 32 * 32 * 4);
  EXPECT_EQ(depth_image.size(), 32 * 32);
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
