#include "drake/systems/sensors/image_io.h"

#include <fstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/sensors/test/image_io_test_params.h"
#include "drake/systems/sensors/test_utilities/image_compare.h"

namespace drake {
namespace systems {
namespace sensors {
namespace internal {
namespace {

namespace fs = std::filesystem;

fs::path GetTempPath(std::string_view filename) {
  static const never_destroyed<std::string> dir{temp_directory()};
  return fs::path(dir.access()) / filename;
}

class ImageIoParameterizedTest
    : public testing::TestWithParam<ImageIoTestParams> {
 public:
  ImageIoParameterizedTest() = default;
};

// Runs Save() and Load() back to back. This is provides a simple sanity check.
TEST_P(ImageIoParameterizedTest, RoundTrip) {
  const ImageIoTestParams& param = GetParam();
  const ImageFileFormat format = param.format;
  const bool write_to_file = param.write_to_file;

  // Create a sample image.
  const ImageAny original = param.CreateIotaDrakeImage();
  const int total_storage = param.total_storage();

  // Call Save() then Load() back to back.
  ImageAny readback;
  if (write_to_file) {
    fs::path path = GetTempPath(fmt::format("RoundTrip_{}.image", format));
    std::visit(
        [&](const auto& image) {
          ImageIo(format).Save(image, path);
        },
        original);
    readback = ImageIo(format).Load(path);
  } else {
    const std::vector<uint8_t> buffer = std::visit(
        [&](const auto& image) {
          return ImageIo(format).Save(image);
        },
        original);
    const ImageIo::ByteSpan span{buffer.data(), buffer.size()};
    readback = ImageIo(format).Load(span);
  }

  // The PixelType must remain intact across the Save + Load.
  ASSERT_EQ(original.index(), readback.index());

  // The number of pixels must remain intact across the Save + Load.
  const int readback_size = std::visit(
      [&](const auto& image) {
        return image.size();
      },
      readback);
  ASSERT_EQ(readback_size, total_storage);

  // Compare the full pixel array for the two images.
  const void* const original_buffer = std::visit(
      [&](const auto& image) {
        const void* buffer = image.at(0, 0);
        return buffer;
      },
      original);
  const void* const readback_buffer = std::visit(
      [&](const auto& image) {
        const void* buffer = image.at(0, 0);
        return buffer;
      },
      readback);
  param.CompareImageBuffer(original_buffer, readback_buffer);
}

// Sanity check that Load() can fill in an image pointer.
TEST_P(ImageIoParameterizedTest, RoundTripPreAllocated) {
  const ImageIoTestParams& param = GetParam();
  if (!param.write_to_file) {
    // Not much is gained from running this test case on both memory buffers as
    // well as files. Simplify our life by skipping the memory buffer cases.
    return;
  }
  const ImageFileFormat format = param.format;

  // Create a sample image.
  const ImageAny original = param.CreateIotaDrakeImage();

  // Call Save().
  const fs::path path =
      GetTempPath(fmt::format("RoundTripPreAllocated_{}.image", format));
  std::visit(
      [&](const auto& image) {
        ImageIo(format).Save(image, path);
      },
      original);

  // Pre-allocate the readback image (filled with zeros).
  ImageAny readback = std::visit(
      [&](const auto& image) {
        using SomeImage = decltype(image);
        return ImageAny{SomeImage{}};
      },
      original);
  DRAKE_DEMAND(original.index() == readback.index());

  // Call Load();
  std::visit(
      [&](auto& image) {
        ImageIo(format).Load(path, &image);
      },
      readback);

  // The PixelType should have remained intact.
  EXPECT_EQ(original.index(), readback.index());

  // The readback should have non-zero contents.
  std::visit(
      [&](auto& image) {
        const int readback_width = image.width();
        const int readback_height = image.height();
        const int readback_size = image.size();
        EXPECT_EQ(readback_width, param.width());
        EXPECT_EQ(readback_height, param.height());
        if (readback_size > 0) {
          EXPECT_NE(image.at(0, 0)[readback_size - 1], 0);
        }
      },
      readback);
}

INSTANTIATE_TEST_SUITE_P(All, ImageIoParameterizedTest,
                         GetAllImageIoTestParams());

// Configuration of the file format.
GTEST_TEST(ImageIoTest, FileFormat) {
  EXPECT_EQ(ImageIo{ImageFileFormat::kTiff}.GetFileFormat(),
            ImageFileFormat::kTiff);

  ImageIo dut;
  EXPECT_EQ(dut.GetFileFormat(), std::nullopt);

  dut.SetFileFormat(ImageFileFormat::kJpeg);
  EXPECT_EQ(dut.GetFileFormat(), ImageFileFormat::kJpeg);

  dut.SetFileFormat(std::nullopt);
  EXPECT_EQ(dut.GetFileFormat(), std::nullopt);
}

// We can save to an output memory buffer, or a buffer return value.
// the format.
GTEST_TEST(ImageIoTest, SaveToOutputArgumentMatchesReturnValue) {
  const ImageIo dut{ImageFileFormat::kPng};
  const ImageRgba8U image(1, 1);
  std::vector<uint8_t> save_output;
  dut.Save(image, &save_output);
  EXPECT_EQ(save_output, dut.Save(image));
}

// Saving without first setting the ImageFileFormat uses the extension to infer
// the format.
GTEST_TEST(ImageIoTest, SaveToFileInfersFormat) {
  const ImageRgba8U image(1, 1);
  fs::path path;
  ImageFileFormat expected;

  expected = ImageFileFormat::kPng;
  path = GetTempPath("SaveToFileInfersFormat.png");
  ImageIo{}.Save(image, path);
  EXPECT_EQ(ImageIo{}.LoadMetadata(path).value().format, expected);

  expected = ImageFileFormat::kJpeg;
  path = GetTempPath("SaveToFileInfersFormat.jpg");
  ImageIo{}.Save(image, path);
  EXPECT_EQ(ImageIo{}.LoadMetadata(path).value().format, expected);
}

// Saving without first setting the ImageFileFormat fails when there is no
// extension.
GTEST_TEST(ImageIoTest, SaveToFileNoExensionNoFormatError) {
  const ImageRgba8U image(1, 1);
  const fs::path path = GetTempPath("SaveToFileNoExensionNoFormatError");
  DRAKE_EXPECT_THROWS_MESSAGE(ImageIo{}.Save(image, path),
                              ".*path does not imply.*");
}

// Saving to memory without first setting the ImageFileFormat fails.
GTEST_TEST(ImageIoTest, SaveToMemoryNoFormatError) {
  const ImageRgba8U image(1, 1);
  DRAKE_EXPECT_THROWS_MESSAGE(ImageIo{}.Save(image),
                              ".*buffer requires SetFileFormat.*");
}

// We can load metadata from a memory buffer.
GTEST_TEST(ImageIoTest, LoadMetdataBuffer) {
  std::vector<uint8_t> saved;
  ImageIo{ImageFileFormat::kPng}.Save(ImageRgba8U(3, 2), &saved);
  const ImageIo::ByteSpan buffer{saved.data(), saved.size()};
  const std::optional<ImageIo::Metadata> meta = ImageIo{}.LoadMetadata(buffer);
  ASSERT_TRUE(meta.has_value());
  EXPECT_EQ(meta->format, ImageFileFormat::kPng);
  EXPECT_EQ(meta->width, 3);
  EXPECT_EQ(meta->height, 2);
}

// Loading from a missing path fails.
GTEST_TEST(ImageIoTest, LoadNoSuchFileError) {
  const fs::path path = GetTempPath("LoadNoSuchFileError.png");
  EXPECT_EQ(ImageIo{}.LoadMetadata(path), std::nullopt);
  DRAKE_EXPECT_THROWS_MESSAGE(ImageIo{}.Load(path),
                              ".*LoadNoSuchFileError.*open.*");
}

// Loading a non-image file fails.
GTEST_TEST(ImageIoTest, LoadNotAnImageError) {
  const fs::path path = GetTempPath("LoadNotAnImageError.png");
  {
    std::ofstream out(path);
    out << "Hello\n";
  }
  EXPECT_EQ(ImageIo{}.LoadMetadata(path), std::nullopt);
  DRAKE_EXPECT_THROWS_MESSAGE(ImageIo{}.Load(path),
                              ".*LoadNotAnImageError.*header.*");
}

// Loading into the wrong pre-allocated Image scalar type fails.
GTEST_TEST(ImageIoTest, LoadWrongPrealloatedScalarError) {
  const ImageIo dut{ImageFileFormat::kPng};
  const std::vector<uint8_t> saved = dut.Save(ImageDepth16U(1, 1));
  const ImageIo::ByteSpan buffer{saved.data(), saved.size()};
  ImageRgba8U color;
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Load(buffer, &color),
                              ".*scalar=16U into scalar=8U.*");
}

// Loading into the wrong pre-allocated Image number of channels fails.
GTEST_TEST(ImageIoTest, LoadWrongPrealloatedNumChannelsError) {
  const ImageIo dut{ImageFileFormat::kPng};
  const std::vector<uint8_t> saved = dut.Save(ImageGrey8U(1, 1));
  const ImageIo::ByteSpan buffer{saved.data(), saved.size()};
  ImageRgb8U rgb;
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Load(buffer, &rgb),
                              ".*channels=1.*channels=3.*");
}

// Loading an RGB image with 16-bit channels is not supported, because there is
// no Image<> template defined to meet that case.
GTEST_TEST(ImageIoTest, LoadUnsupportedChannelScalarError) {
  const fs::path path{
      FindResourceOrThrow("drake/systems/sensors/test/png_color16_test.png")};
  const ImageIo dut{ImageFileFormat::kPng};
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Load(path), ".*template instantiation.*");
}

GTEST_TEST(ImageIoTest, LoadUnsupportedColorTiffError) {
  const fs::path path{
      FindResourceOrThrow("drake/systems/sensors/test/tiff_rgb32f_test.tif")};
  const ImageIo dut{ImageFileFormat::kTiff};
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Load(path), ".*template instantiation.*");
}

// For legacy reasons, we allow type-punning from unsigned 16-bit image data
// into Drake's signed 16-bit image channel. We intend to eventually deprecate
// and remove this (by fixing our label images to use unsigned numbers) but for
// now we need to ensure this capability remains intact.
GTEST_TEST(ImageIoTest, Load16UDataAs16I) {
  const ImageIo dut{ImageFileFormat::kPng};

  const ImageDepth16U depth(1, 1, uint16_t{22});
  const std::vector<uint8_t> saved = dut.Save(depth);
  const ImageIo::ByteSpan buffer{saved.data(), saved.size()};

  ImageLabel16I label;
  dut.Load(buffer, &label);

  ASSERT_EQ(label.width(), 1);
  ASSERT_EQ(label.height(), 1);
  ASSERT_EQ(label.at(0, 0)[0], 22);
}

// Loading can automatically drop an unwanted alpha channel.
GTEST_TEST(ImageIoTest, LoadDropAlpha) {
  const ImageIo dut{ImageFileFormat::kPng};
  const ImageRgba8U rgba(3, 2, uint8_t{22});
  const std::vector<uint8_t> saved = dut.Save(rgba);
  const ImageIo::ByteSpan buffer{saved.data(), saved.size()};
  ImageRgb8U rgb;
  dut.Load(buffer, &rgb);
  ASSERT_EQ(rgb.width(), 3);
  ASSERT_EQ(rgb.height(), 2);
  EXPECT_EQ(rgb.at(0, 0)[0], 22);
}

// Loading can automatically add a alpha channel.
GTEST_TEST(ImageIoTest, LoadAddAlpha) {
  const ImageIo dut{ImageFileFormat::kPng};
  const ImageRgb8U rgb(3, 2, uint8_t{22});
  const std::vector<uint8_t> saved = dut.Save(rgb);
  const ImageIo::ByteSpan buffer{saved.data(), saved.size()};
  ImageRgba8U rgba;
  dut.Load(buffer, &rgba);
  ASSERT_EQ(rgba.width(), 3);
  ASSERT_EQ(rgba.height(), 2);
  EXPECT_EQ(rgba.at(0, 0)[0], 22);
  EXPECT_EQ(rgba.at(0, 0)[3], 0xff);
}

}  // namespace
}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
