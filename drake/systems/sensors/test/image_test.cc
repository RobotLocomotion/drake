#include "drake/systems/sensors/image.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace sensors {
namespace {
const int kWidth = 640;
const int kHeight = 480;
const int kChannel = 4;
const uint8_t kInitialValue = 100;

GTEST_TEST(TestImage, EmptyTest) {
  Image<uint8_t> dut;

  EXPECT_EQ(dut.width(), 0);
  EXPECT_EQ(dut.height(), 0);
  EXPECT_EQ(dut.size(), 0);
  EXPECT_EQ(dut.num_channels(), 0);
}

GTEST_TEST(TestImage, InstantiateTest) {
  Image<uint8_t> dut(kWidth, kHeight, kChannel);

  EXPECT_EQ(dut.width(), kWidth);
  EXPECT_EQ(dut.height(), kHeight);
  EXPECT_EQ(dut.num_channels(), kChannel);
}

GTEST_TEST(TestImage, InitializeAndAccessToPixelValuesTest) {
  // If you don't give initial value, the default value is zero.
  Image<uint8_t> dut(kWidth, kHeight, kChannel);
  Image<uint8_t> dut2(kWidth, kHeight, kChannel, kInitialValue);

  for (int u = 0; u < kWidth; ++u) {
    for (int v = 0; v < kHeight; ++v) {
      for (int channel = 0; channel < kChannel; ++channel) {
        EXPECT_EQ(dut.at(u, v)[channel], 0);
        EXPECT_EQ(dut2.at(u, v)[channel], kInitialValue);
      }
    }
  }
}


GTEST_TEST(TestImage, CopyConstructorTest) {
  Image<uint8_t> image(kWidth, kHeight, kChannel, kInitialValue);
  Image<uint8_t> dut(image);
  Image<uint8_t> dut2 = image;

  EXPECT_EQ(dut.width(), image.width());
  EXPECT_EQ(dut.height(), image.height());
  EXPECT_EQ(dut.num_channels(), image.num_channels());

  EXPECT_EQ(dut2.width(), image.width());
  EXPECT_EQ(dut2.height(), image.height());
  EXPECT_EQ(dut2.num_channels(), image.num_channels());

  for (int u = 0; u < kWidth; ++u) {
    for (int v = 0; v < kHeight; ++v) {
      for (int channel = 0; channel < kChannel; ++channel) {
        EXPECT_EQ(dut.at(u, v)[channel], image.at(u, v)[channel]);
        EXPECT_EQ(dut2.at(u, v)[channel], image.at(u, v)[channel]);
      }
    }
  }
}

GTEST_TEST(TestImage, AssignmentOperatorTest) {
  Image<uint8_t> image(kWidth, kHeight, kChannel, kInitialValue);
  Image<uint8_t> dut(1, 1, 1);
  dut = image;

  EXPECT_EQ(dut.width(), image.width());
  EXPECT_EQ(dut.height(), image.height());
  EXPECT_EQ(dut.num_channels(), image.num_channels());

  for (int u = 0; u < kWidth; ++u) {
    for (int v = 0; v < kHeight; ++v) {
      for (int channel = 0; channel < kChannel; ++channel) {
        EXPECT_EQ(dut.at(u, v)[channel], image.at(u, v)[channel]);
      }
    }
  }
}

GTEST_TEST(TestImage, MoveConstructorTest) {
  Image<uint8_t> image(kWidth, kHeight, kChannel, kInitialValue);
  Image<uint8_t> dut(std::move(image));

  EXPECT_EQ(dut.width(), kWidth);
  EXPECT_EQ(dut.height(), kHeight);
  EXPECT_EQ(dut.num_channels(), kChannel);

  EXPECT_EQ(image.width(), 0);
  EXPECT_EQ(image.height(), 0);
  EXPECT_EQ(image.num_channels(), 0);

  for (int u = 0; u < kWidth; ++u) {
    for (int v = 0; v < kHeight; ++v) {
      for (int channel = 0; channel < kChannel; ++channel) {
        EXPECT_EQ(dut.at(u, v)[channel], kInitialValue);
      }
    }
  }
}

GTEST_TEST(TestImage, MoveAssignmentOperatorTest) {
  Image<uint8_t> image(kWidth, kHeight, kChannel, kInitialValue);
  Image<uint8_t> dut(kWidth / 2, kHeight / 2, 1);

  dut = std::move(image);

  EXPECT_EQ(dut.width(), kWidth);
  EXPECT_EQ(dut.height(), kHeight);
  EXPECT_EQ(dut.num_channels(), kChannel);

  EXPECT_EQ(image.width(), 0);
  EXPECT_EQ(image.height(), 0);
  EXPECT_EQ(image.num_channels(), 0);

  for (int u = 0; u < kWidth; ++u) {
    for (int v = 0; v < kHeight; ++v) {
      for (int channel = 0; channel < kChannel; ++channel) {
        EXPECT_EQ(dut.at(u, v)[channel], kInitialValue);
      }
    }
  }
}

GTEST_TEST(TestImage, ResizeTest) {
  Image<uint8_t> dut(kWidth, kHeight, kChannel);
  const int kWidthResized = 64;
  const int kHeightResized = 48;
  dut.resize(kWidthResized, kHeightResized);

  EXPECT_EQ(dut.width(), kWidthResized);
  EXPECT_EQ(dut.height(), kHeightResized);
  EXPECT_EQ(dut.num_channels(), kChannel);
  EXPECT_EQ(dut.size(), kWidthResized * kHeightResized * kChannel);
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
