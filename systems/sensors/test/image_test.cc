#include "drake/systems/sensors/image.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace sensors {
namespace {

const int kWidth = 640;
const int kHeight = 480;
const uint8_t kInitialValue = 100;

GTEST_TEST(TestImage, EmptyTest) {
  ImageRgb8U dut;

  EXPECT_EQ(dut.width(), 0);
  EXPECT_EQ(dut.height(), 0);
  EXPECT_EQ(dut.size(), 0);
  EXPECT_EQ(dut.kNumChannels, 3);
  EXPECT_EQ(dut.kPixelSize, 3);
  EXPECT_EQ(dut.kPixelFormat, PixelFormat::kRgb);
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
  dut.resize(kWidthResized, kHeightResized);

  EXPECT_EQ(dut.width(), kWidthResized);
  EXPECT_EQ(dut.height(), kHeightResized);
  EXPECT_EQ(dut.kNumChannels, kNumChannels);
  EXPECT_EQ(dut.kPixelFormat, PixelFormat::kDepth);
  EXPECT_EQ(dut.kPixelSize, 2);

  EXPECT_EQ(dut.size(), kWidthResized * kHeightResized * kNumChannels);
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
