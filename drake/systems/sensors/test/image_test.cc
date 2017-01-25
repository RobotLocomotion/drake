#include "drake/systems/sensors/image.h"

#include "gtest/gtest.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {
const int kWidth = 640;
const int kHeight = 480;
const int kChannel = 4;
const uint8_t kInitialValue = 100;

GTEST_TEST(TestImage, InstantiateTest) {
  Image<uint8_t, kChannel> dut(kWidth, kHeight);

  EXPECT_EQ(dut.width(), kWidth);
  EXPECT_EQ(dut.height(), kHeight);
  EXPECT_EQ(dut.num_channels(), kChannel);
}

GTEST_TEST(TestImage, InitializeAndAccessToPixelValuesTest) {
  // If you don't give initial value, the default value is zero.
  Image<uint8_t, kChannel> dut(kWidth, kHeight);
  Image<uint8_t, kChannel> dut2(kWidth, kHeight, kInitialValue);

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
  Image<uint8_t, kChannel> image(kWidth, kHeight, kInitialValue);
  Image<uint8_t, kChannel> dut(image);
  Image<uint8_t, kChannel> dut2 = image;

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
  Image<uint8_t, kChannel> image(kWidth, kHeight, kInitialValue);
  Image<uint8_t, kChannel> dut(1, 1);
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
  Image<uint8_t, kChannel> image(kWidth, kHeight, kInitialValue);
  Image<uint8_t, kChannel> dut(std::move(image));

  EXPECT_EQ(dut.width(), kWidth);
  EXPECT_EQ(dut.height(), kHeight);
  EXPECT_EQ(dut.num_channels(), kChannel);

  EXPECT_EQ(image.width(), 0);
  EXPECT_EQ(image.height(), 0);
  EXPECT_EQ(image.num_channels(), kChannel);

  for (int u = 0; u < kWidth; ++u) {
    for (int v = 0; v < kHeight; ++v) {
      for (int channel = 0; channel < kChannel; ++channel) {
        EXPECT_EQ(dut.at(u, v)[channel], kInitialValue);
      }
    }
  }
}

GTEST_TEST(TestImage, MoveAssignmentOperatorTest) {
  Image<uint8_t, kChannel> image(kWidth, kHeight, kInitialValue);
  Image<uint8_t, kChannel> dut(kWidth / 2, kHeight / 2);

  dut = std::move(image);

  EXPECT_EQ(dut.width(), kWidth);
  EXPECT_EQ(dut.height(), kHeight);
  EXPECT_EQ(dut.num_channels(), kChannel);

  EXPECT_EQ(image.width(), 0);
  EXPECT_EQ(image.height(), 0);
  EXPECT_EQ(image.num_channels(), kChannel);

  for (int u = 0; u < kWidth; ++u) {
    for (int v = 0; v < kHeight; ++v) {
      for (int channel = 0; channel < kChannel; ++channel) {
        EXPECT_EQ(dut.at(u, v)[channel], kInitialValue);
      }
    }
  }
}

GTEST_TEST(TestImage, ResizeTest) {
  Image<uint8_t, kChannel> dut(kWidth, kHeight);
  const int kWidthResized = 64;
  const int kHeightResized = 48;
  dut.resize(kWidthResized, kHeightResized);

  EXPECT_EQ(dut.width(), kWidthResized);
  EXPECT_EQ(dut.height(), kHeightResized);
  EXPECT_EQ(dut.num_channels(), kChannel);
  EXPECT_EQ(dut.size(), kWidthResized * kHeightResized * kChannel);
}

GTEST_TEST(TestImage, ResizeInvalidWidthTest) {
  Image<uint8_t, kChannel> dut(kWidth, kHeight);

  const int kInvalidWidth = 0;
  const int kHeightResized = 48;
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH(dut.resize(kInvalidWidth, kHeightResized), ".*");
}

GTEST_TEST(TestImage, ResizeInvalidHeightTest) {
  Image<uint8_t, kChannel> dut(kWidth, kHeight);

  const int kWidthResized = 64;
  const int kInvalidHeight = 0;
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH(dut.resize(kWidthResized, kInvalidHeight), ".*");
}

GTEST_TEST(TestImage, AtNegativeHeightTest) {
  Image<uint8_t, kChannel> dut(kWidth, kHeight);

  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_DEATH(dut.at(kWidth-1, -1), ".*");
}

GTEST_TEST(TestImage, AtTooLargeHeightTest) {
  Image<uint8_t, kChannel> dut(kWidth, kHeight);

  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_DEATH(dut.at(kWidth-1, kHeight), ".*");
}

GTEST_TEST(TestImage, AtNegativeWidthTest) {
  Image<uint8_t, kChannel> dut(kWidth, kHeight);

  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_DEATH(dut.at(-1, kHeight-1), ".*");
}

GTEST_TEST(TestImage, AtTooLargeWidthTest) {
  Image<uint8_t, kChannel> dut(kWidth, kHeight);

  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_DEATH(dut.at(kWidth, kHeight-1), ".*");
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
