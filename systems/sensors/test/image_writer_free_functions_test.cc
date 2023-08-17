#include <filesystem>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/temp_directory.h"
#include "drake/systems/sensors/image_writer.h"
#include "drake/systems/sensors/test_utilities/image_compare.h"

namespace drake {
namespace systems {
namespace sensors {

namespace fs = std::filesystem;

namespace {

class SaveImageTest : public ::testing::Test {
 public:
  void SetUp() override {
    const std::string name =
        testing::UnitTest::GetInstance()->current_test_info()->name();
    filename_ =
        (fs::path(temp_directory()) / fmt::format("{}.png", name)).string();
  }

  template <PixelType kPixelType>
  void CheckReadback(const Image<kPixelType>& expected) {
    Image<kPixelType> readback;
    ASSERT_TRUE(LoadImage(filename_, &readback));
    EXPECT_EQ(readback, expected);
  }

 protected:
  std::string filename_;
};

// Saves a simple 4x1 image consisting of: [red][green][blue][white].
TEST_F(SaveImageTest, SaveToPng_Color) {
  Image<PixelType::kRgba8U> image(4, 1);
  auto set_color = [&image](int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    image.at(x, y)[0] = r;
    image.at(x, y)[1] = g;
    image.at(x, y)[2] = b;
    image.at(x, y)[3] = 255;
  };
  set_color(0, 0, 255, 0, 0);
  set_color(1, 0, 0, 255, 0);
  set_color(2, 0, 0, 0, 255);
  set_color(3, 0, 255, 255, 255);
  EXPECT_NO_THROW(SaveToPng(image, filename_));
  CheckReadback(image);
}

// Saves a simple 4x1 image consisting of: 0, 0.25, 0.5, 0.75
TEST_F(SaveImageTest, SaveToTiff_Depth) {
  Image<PixelType::kDepth32F> image(4, 1);
  *image.at(0, 0) = 0.0f;
  *image.at(1, 0) = 0.25f;
  *image.at(2, 0) = 0.5f;
  *image.at(3, 0) = 1.0f;
  EXPECT_NO_THROW(SaveToTiff(image, filename_));
  CheckReadback(image);
}

// Saves a simple 4x1 image consisting of: 0, 100, 200, 300.
// Note: value > 255 to make sure that values aren't being truncated/wrapped
// to 8-bit values.
TEST_F(SaveImageTest, SaveToPng_Label) {
  Image<PixelType::kLabel16I> image(4, 1);
  *image.at(0, 0) = 0;
  *image.at(1, 0) = 100;
  *image.at(2, 0) = 200;
  *image.at(3, 0) = 300;
  EXPECT_NO_THROW(SaveToPng(image, filename_));
  CheckReadback(image);
}

// Saves a simple 4x1 image consisting of: 0, 100, 200, 300.
// Note: value > 255 to make sure that values aren't being truncated/wrapped
// to 8-bit values.
TEST_F(SaveImageTest, SaveToPng_Depth16) {
  Image<PixelType::kDepth16U> image(4, 1);
  *image.at(0, 0) = 0;
  *image.at(1, 0) = 100;
  *image.at(2, 0) = 200;
  *image.at(3, 0) = 300;
  EXPECT_NO_THROW(SaveToPng(image, filename_));
  CheckReadback(image);
}

// Saves a simple 4x1 image consisting of: 1, 2, 3, 4.
TEST_F(SaveImageTest, SaveToPng_Grey) {
  Image<PixelType::kGrey8U> image(4, 1);
  *image.at(0, 0) = 1;
  *image.at(1, 0) = 2;
  *image.at(2, 0) = 3;
  *image.at(3, 0) = 4;
  EXPECT_NO_THROW(SaveToPng(image, filename_));
  CheckReadback(image);
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
