#include "drake/systems/sensors/image.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace sensors {
namespace {

const int kWidth = 640;
const int kHeight = 480;
const uint8_t kInitialValue = 100;

GTEST_TEST(TestImage, EmptyTest0) {
  ImageRgb8U dut;

  EXPECT_EQ(dut.width(), 0);
  EXPECT_EQ(dut.height(), 0);
  EXPECT_EQ(dut.size(), 0);
  EXPECT_EQ(dut.kNumChannels, 3);
  EXPECT_EQ(dut.kPixelSize, 3);
  EXPECT_EQ(dut.kPixelFormat, PixelFormat::kRgb);
}

GTEST_TEST(TestImage, EmptyTest2) {
  ImageRgb8U dut(0, 0);
  EXPECT_EQ(dut.width(), 0);
  EXPECT_EQ(dut.height(), 0);
  EXPECT_EQ(dut.size(), 0);

  EXPECT_THROW(ImageRgb8U(0, 1), std::exception);
  EXPECT_THROW(ImageRgb8U(1, 0), std::exception);
  EXPECT_THROW(ImageRgb8U(-1, 1), std::exception);
  EXPECT_THROW(ImageRgb8U(1, -1), std::exception);
}

GTEST_TEST(TestImage, EmptyTest3) {
  ImageRgb8U dut(0, 0, 0);
  EXPECT_EQ(dut.width(), 0);
  EXPECT_EQ(dut.height(), 0);
  EXPECT_EQ(dut.size(), 0);

  EXPECT_THROW(ImageRgb8U(0, 1, 0), std::exception);
  EXPECT_THROW(ImageRgb8U(1, 0, 0), std::exception);
  EXPECT_THROW(ImageRgb8U(-1, 1, 0), std::exception);
  EXPECT_THROW(ImageRgb8U(1, -1, 0), std::exception);
}

GTEST_TEST(TestImage, InstantiateTest) {
  ImageBgr8U dut(kWidth, kHeight);
  const int kNumChannels = 3;

  EXPECT_EQ(dut.width(), kWidth);
  EXPECT_EQ(dut.height(), kHeight);
  EXPECT_EQ(dut.kNumChannels, kNumChannels);
  EXPECT_EQ(dut.kPixelSize, 3);
  EXPECT_EQ(dut.size(), kWidth * kHeight * kNumChannels);
  EXPECT_EQ(dut.kPixelFormat, PixelFormat::kBgr);
}

GTEST_TEST(TestImage, InitializeAndAccessToPixelValuesTest) {
  // If you don't give initial value, the default value is zero.
  ImageRgba8U dut(kWidth, kHeight);
  ImageRgba8U dut2(kWidth, kHeight, kInitialValue);

  for (int u = 0; u < kWidth; ++u) {
    for (int v = 0; v < kHeight; ++v) {
      for (int channel = 0; channel < dut.kNumChannels; ++channel) {
        EXPECT_EQ(dut.at(u, v)[channel], 0);
        EXPECT_EQ(dut2.at(u, v)[channel], kInitialValue);
      }
    }
  }
  EXPECT_EQ(dut.kPixelFormat, PixelFormat::kRgba);
  EXPECT_EQ(dut.kPixelSize, 4);
  EXPECT_EQ(dut2.kPixelFormat, PixelFormat::kRgba);
}

GTEST_TEST(TestImage, CopyConstructorTest) {
  ImageBgra8U image(kWidth, kHeight, kInitialValue);
  ImageBgra8U dut(image);
  ImageBgra8U dut2 = image;

  EXPECT_EQ(dut.width(), image.width());
  EXPECT_EQ(dut.height(), image.height());
  EXPECT_EQ(dut.kNumChannels, image.kNumChannels);
  EXPECT_EQ(dut.kPixelSize, 4);

  EXPECT_EQ(dut2.width(), image.width());
  EXPECT_EQ(dut2.height(), image.height());
  EXPECT_EQ(dut2.kNumChannels, image.kNumChannels);
  EXPECT_EQ(dut2.kPixelSize, 4);

  EXPECT_EQ(image.kPixelFormat, PixelFormat::kBgra);
  EXPECT_EQ(dut.kPixelFormat, PixelFormat::kBgra);
  EXPECT_EQ(dut2.kPixelFormat, PixelFormat::kBgra);

  for (int u = 0; u < kWidth; ++u) {
    for (int v = 0; v < kHeight; ++v) {
      for (int channel = 0; channel < image.kNumChannels; ++channel) {
        EXPECT_EQ(dut.at(u, v)[channel], image.at(u, v)[channel]);
        EXPECT_EQ(dut2.at(u, v)[channel], image.at(u, v)[channel]);
      }
    }
  }
}

GTEST_TEST(TestImage, AssignmentOperatorTest) {
  ImageDepth32F image(kWidth, kHeight, kInitialValue);
  ImageDepth32F dut(1, 1);
  dut = image;

  EXPECT_EQ(dut.width(), image.width());
  EXPECT_EQ(dut.height(), image.height());
  EXPECT_EQ(dut.kNumChannels, image.kNumChannels);
  EXPECT_EQ(dut.kPixelFormat, PixelFormat::kDepth);
  EXPECT_EQ(dut.kPixelSize, 4);
  EXPECT_EQ(image.kPixelFormat, PixelFormat::kDepth);

  for (int u = 0; u < kWidth; ++u) {
    for (int v = 0; v < kHeight; ++v) {
      for (int channel = 0; channel < image.kNumChannels; ++channel) {
        EXPECT_EQ(dut.at(u, v)[channel], image.at(u, v)[channel]);
      }
    }
  }
}

GTEST_TEST(TestImage, MoveConstructorTest) {
  ImageLabel16I image(kWidth, kHeight, kInitialValue);
  ImageLabel16I dut(std::move(image));

  const int kNumChannels = 1;

  EXPECT_EQ(dut.width(), kWidth);
  EXPECT_EQ(dut.height(), kHeight);
  EXPECT_EQ(dut.kNumChannels, kNumChannels);
  EXPECT_EQ(dut.kPixelFormat, PixelFormat::kLabel);
  EXPECT_EQ(dut.kPixelSize, 2);

  EXPECT_EQ(image.width(), 0);
  EXPECT_EQ(image.height(), 0);
  EXPECT_EQ(image.kNumChannels, kNumChannels);
  EXPECT_EQ(image.kPixelFormat, PixelFormat::kLabel);

  for (int u = 0; u < kWidth; ++u) {
    for (int v = 0; v < kHeight; ++v) {
      for (int channel = 0; channel < image.kNumChannels; ++channel) {
        EXPECT_EQ(dut.at(u, v)[channel], kInitialValue);
      }
    }
  }
}

GTEST_TEST(TestImage, MoveAssignmentOperatorTest) {
  ImageGrey8U image(kWidth, kHeight, kInitialValue);
  ImageGrey8U dut(kWidth / 2, kHeight / 2);

  dut = std::move(image);
  const int kNumChannels = 1;

  EXPECT_EQ(dut.width(), kWidth);
  EXPECT_EQ(dut.height(), kHeight);
  EXPECT_EQ(dut.kNumChannels, kNumChannels);
  EXPECT_EQ(dut.kPixelFormat, PixelFormat::kGrey);
  EXPECT_EQ(dut.kPixelSize, 1);

  EXPECT_EQ(image.width(), 0);
  EXPECT_EQ(image.height(), 0);
  EXPECT_EQ(image.kNumChannels, kNumChannels);
  EXPECT_EQ(image.kPixelFormat, PixelFormat::kGrey);

  for (int u = 0; u < kWidth; ++u) {
    for (int v = 0; v < kHeight; ++v) {
      for (int channel = 0; channel < image.kNumChannels; ++channel) {
        EXPECT_EQ(dut.at(u, v)[channel], kInitialValue);
      }
    }
  }
}

GTEST_TEST(TestImage, ResizeTest) {
  ImageDepth16U dut(kWidth, kHeight);
  const int kWidthResized = 64;
  const int kHeightResized = 48;
  const int kNumChannels = 1;

  // Resize to non-zero.
  dut.resize(kWidthResized, kHeightResized);
  EXPECT_EQ(dut.width(), kWidthResized);
  EXPECT_EQ(dut.height(), kHeightResized);
  EXPECT_EQ(dut.kNumChannels, kNumChannels);
  EXPECT_EQ(dut.kPixelFormat, PixelFormat::kDepth);
  EXPECT_EQ(dut.kPixelSize, 2);
  EXPECT_EQ(dut.size(), kWidthResized * kHeightResized * kNumChannels);

  // Invalid resizes.
  EXPECT_THROW(dut.resize(0, 1), std::exception);
  EXPECT_THROW(dut.resize(1, 0), std::exception);
  EXPECT_THROW(dut.resize(-1, 1), std::exception);
  EXPECT_THROW(dut.resize(1, -1), std::exception);

  // Resize to zero.
  dut.resize(0, 0);
  EXPECT_EQ(dut.width(), 0);
  EXPECT_EQ(dut.height(), 0);
  EXPECT_EQ(dut.size(), 0);
}

GTEST_TEST(ImageTest, DepthImage32FTo16U) {
  // Create a list of test inputs and outputs (pixels).
  using InPixel = ImageDepth32F::Traits;
  using OutPixel = ImageDepth16U::Traits;
  const std::vector<std::pair<float, uint16_t>> test_cases{
      // Too close or too far.
      {InPixel::kTooClose, OutPixel::kTooClose},
      {InPixel::kTooFar, OutPixel::kTooFar},
      // Valid distances for both pixel types.
      {3.0f, 3000},
      {10.0f, 10000},
      // Valid distance for input pixel, but saturates the output pixel.
      {100.0f, OutPixel::kTooFar},
      // Input pixels approaching the saturation point of the output pixel.
      {65.531f, 65531},
      {65.534001f, 65534},
      {65.535f, 65535},
      {65.536f, OutPixel::kTooFar},
      {65.537f, OutPixel::kTooFar},
      // Crazy input value.
      {-1.0f, OutPixel::kTooClose},
      // Special values.
      {-std::numeric_limits<float>::infinity(), OutPixel::kTooClose},
      {std::numeric_limits<float>::infinity(), OutPixel::kTooFar},
      {std::numeric_limits<float>::quiet_NaN(), OutPixel::kTooFar},
  };

  // Check each pair of input and output pixel values.
  for (const auto& [pixel_in, pixel_out] : test_cases) {
    SCOPED_TRACE(fmt::format("pixel_in = {}", pixel_in));
    ImageDepth32F image_in(1, 1);
    ImageDepth16U image_out(1, 1);
    image_in.at(0, 0)[0] = pixel_in;
    ConvertDepth32FTo16U(image_in, &image_out);
    ASSERT_EQ(image_out.width(), 1);
    ASSERT_EQ(image_out.height(), 1);
    EXPECT_EQ(image_out.at(0, 0)[0], pixel_out);
  }

  // Check that height and width auto-resizing is correct.
  ImageDepth32F image_in(3, 4);
  ImageDepth16U image_out;
  ConvertDepth32FTo16U(image_in, &image_out);
  EXPECT_EQ(image_out.width(), image_in.width());
  EXPECT_EQ(image_out.height(), image_in.height());
  image_in.resize(0, 0);
  ConvertDepth32FTo16U(image_in, &image_out);
  EXPECT_EQ(image_out.width(), 0);
  EXPECT_EQ(image_out.height(), 0);
}

GTEST_TEST(ImageTest, DepthImage16UTo32F) {
  // Create a list of test inputs and outputs (pixels).
  using InPixel = ImageDepth16U::Traits;
  using OutPixel = ImageDepth32F::Traits;
  const std::vector<std::pair<uint16_t, float>> test_cases{
      // Too close or too far.
      {InPixel::kTooClose, OutPixel::kTooClose},
      {InPixel::kTooFar, OutPixel::kTooFar},
      // Valid distances for both pixel types.
      {3000, 3.0f},
      {10000, 10.0f},
  };

  // Check each pair of input and output pixel values.
  for (const auto& [pixel_in, pixel_out] : test_cases) {
    SCOPED_TRACE(fmt::format("pixel_in = {}", pixel_in));
    ImageDepth16U image_in(1, 1);
    ImageDepth32F image_out(1, 1);
    image_in.at(0, 0)[0] = pixel_in;
    ConvertDepth16UTo32F(image_in, &image_out);
    ASSERT_EQ(image_out.width(), 1);
    ASSERT_EQ(image_out.height(), 1);
    EXPECT_EQ(image_out.at(0, 0)[0], pixel_out);
  }

  // Check that height and width auto-resizing is correct.
  ImageDepth16U image_in(3, 4);
  ImageDepth32F image_out;
  ConvertDepth16UTo32F(image_in, &image_out);
  EXPECT_EQ(image_out.width(), image_in.width());
  EXPECT_EQ(image_out.height(), image_in.height());
  image_in.resize(0, 0);
  ConvertDepth16UTo32F(image_in, &image_out);
  EXPECT_EQ(image_out.width(), 0);
  EXPECT_EQ(image_out.height(), 0);
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
