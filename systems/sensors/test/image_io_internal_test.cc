#include "drake/systems/sensors/image_io_internal.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"

namespace drake {
namespace systems {
namespace sensors {
namespace internal {
namespace {

namespace fs = std::filesystem;

GTEST_TEST(ImageIoInternalTest, FileFormatFromExtension) {
  // Simple cases.
  EXPECT_EQ(FileFormatFromExtension("foo.jpeg"), ImageFileFormat::kJpeg);
  EXPECT_EQ(FileFormatFromExtension("foo.jpg"), ImageFileFormat::kJpeg);
  EXPECT_EQ(FileFormatFromExtension("foo.png"), ImageFileFormat::kPng);
  EXPECT_EQ(FileFormatFromExtension("foo.tiff"), ImageFileFormat::kTiff);
  EXPECT_EQ(FileFormatFromExtension("foo.tif"), ImageFileFormat::kTiff);

  // Capitalization doesn't matter.
  EXPECT_EQ(FileFormatFromExtension("foo.Jpeg"), ImageFileFormat::kJpeg);
  EXPECT_EQ(FileFormatFromExtension("foo.JPG"), ImageFileFormat::kJpeg);
  EXPECT_EQ(FileFormatFromExtension("foo.Png"), ImageFileFormat::kPng);
  EXPECT_EQ(FileFormatFromExtension("foo.TIFF"), ImageFileFormat::kTiff);
  EXPECT_EQ(FileFormatFromExtension("foo.tiF"), ImageFileFormat::kTiff);

  // Full paths are okay.
  EXPECT_EQ(FileFormatFromExtension("/path/foo.jpeg"), ImageFileFormat::kJpeg);
  EXPECT_EQ(FileFormatFromExtension("/path/foo.jpg"), ImageFileFormat::kJpeg);
  EXPECT_EQ(FileFormatFromExtension("/path/foo.png"), ImageFileFormat::kPng);
  EXPECT_EQ(FileFormatFromExtension("/path/foo.tiff"), ImageFileFormat::kTiff);
  EXPECT_EQ(FileFormatFromExtension("/path/foo.tif"), ImageFileFormat::kTiff);

  // Unknown extensions.
  EXPECT_EQ(FileFormatFromExtension("other.txt"), std::nullopt);
  EXPECT_EQ(FileFormatFromExtension("png"), std::nullopt);
  EXPECT_EQ(FileFormatFromExtension("/png/other.txt"), std::nullopt);
  EXPECT_EQ(FileFormatFromExtension("/foo.png/other.txt"), std::nullopt);
}

GTEST_TEST(ImageIoInternalTest, GuessFileFormatFromBuffer) {
  std::array<uint8_t, 2> header;
  const ImageIo::ByteSpan span{header.data(), header.size()};

  // Valid headers.
  header = {0xff, 0xd8};
  EXPECT_EQ(GuessFileFormat(span), ImageFileFormat::kJpeg);
  header = {0x89, 0x50};
  EXPECT_EQ(GuessFileFormat(span), ImageFileFormat::kPng);
  header = {0x49, 0x49};
  EXPECT_EQ(GuessFileFormat(span), ImageFileFormat::kTiff);
  header = {0x4d, 0x4d};
  EXPECT_EQ(GuessFileFormat(span), ImageFileFormat::kTiff);

  // Invalid headers.
  header = {};
  EXPECT_EQ(GuessFileFormat(span), std::nullopt);
  EXPECT_EQ(GuessFileFormat(ImageIo::ByteSpan{}), std::nullopt);
}

GTEST_TEST(ImageIoInternalTest, GuessFileFormatFromFile) {
  const fs::path jpg_file =
      FindResourceOrThrow("drake/systems/sensors/test/jpeg_test.jpg");
  const fs::path png_file =
      FindResourceOrThrow("drake/systems/sensors/test/png_color_test.png");
  const fs::path tif_file =
      FindResourceOrThrow("drake/systems/sensors/test/tiff_32f_test.tif");

  // Valid headers.
  EXPECT_EQ(GuessFileFormat(&jpg_file), ImageFileFormat::kJpeg);
  EXPECT_EQ(GuessFileFormat(&png_file), ImageFileFormat::kPng);
  EXPECT_EQ(GuessFileFormat(&tif_file), ImageFileFormat::kTiff);

  // Invalid headers.
  const fs::path zero = "/dev/zero";
  const fs::path null = "/dev/null";
  const fs::path none = "/no/such/file";
  EXPECT_EQ(GuessFileFormat(&zero), std::nullopt);
  EXPECT_EQ(GuessFileFormat(&null), std::nullopt);
  EXPECT_EQ(GuessFileFormat(&none), std::nullopt);
}

GTEST_TEST(ImageIoInternalTest, NeedsBgrSwizzle) {
  // This is a trivial constexpr function, so it's sufficient to make sure it's
  // actually `constexpr` with a spot-check. We rely on code review to ensure
  // the switch cases are correct.
  constexpr bool swizzle_bgr = NeedsBgrSwizzle<PixelType::kBgr8U>();
  EXPECT_TRUE(swizzle_bgr);
}

GTEST_TEST(ImageIoInternalTest, GetVtkScalarType) {
  // This is a trivial constexpr function, so it's sufficient to make sure it's
  // actually `constexpr` with a spot-check. We rely on code review to ensure
  // the switch cases are correct.
  constexpr int result = GetVtkScalarType<PixelType::kRgb8U>();
  EXPECT_EQ(result, VTK_TYPE_UINT8);
}

GTEST_TEST(ImageIoInternalTest, GetDrakeScalarType) {
  // This is a trivial constexpr function, so it's sufficient to make sure it's
  // actually `constexpr` with a spot-check. We rely on code review to ensure
  // the switch cases are correct.
  constexpr std::optional<PixelScalar> result =
      GetDrakeScalarType(VTK_TYPE_UINT8);
  EXPECT_EQ(result, PixelScalar::k8U);
}

GTEST_TEST(ImageIoInternalTest, CopyAndFlipRaw) {
  // Choose a distinct value for every argument.
  constexpr int width = 5;
  constexpr int height = 2;
  constexpr int source_channels = 3;
  constexpr int dest_channels = 4;
  constexpr uint8_t dest_pad = 99;

  // Fill the source with incrementing numbers.
  // clang-format off
  constexpr int source_size = width * height * source_channels;
  const std::array<uint8_t, source_size> source = {
       1, 2, 3,  4, 5, 6,  7, 8, 9, 10,11,12, 13,14,15,  // NOLINT
      16,17,18, 19,20,21, 22,23,24, 25,26,27, 28,29,30,  // NOLINT
  };
  // clang-format on

  // Prepare an output buffer.
  constexpr int dest_size = width * height * dest_channels;
  std::array<uint8_t, dest_size> dest;

  // Check without swizzling.
  CopyAndFlipRaw<uint8_t, source_channels, /* swizzle = */ false, dest_channels,
                 dest_pad>(source.data(), dest.data(), width, height);
  // clang-format off
  EXPECT_THAT(dest, testing::ElementsAre(
      16,17,18,99, 19,20,21,99, 22,23,24,99, 25,26,27,99, 28,29,30,99,  // NOLINT
       1, 2, 3,99,  4, 5, 6,99,  7, 8, 9,99, 10,11,12,99, 13,14,15,99   // NOLINT
  ));                                                                   // NOLINT
  // clang-format on

  // Check with swizzling.
  CopyAndFlipRaw<uint8_t, source_channels, /* swizzle = */ true, dest_channels,
                 dest_pad>(source.data(), dest.data(), width, height);
  // clang-format off
  EXPECT_THAT(dest, testing::ElementsAre(
      18,17,16,99, 21,20,19,99, 24,23,22,99, 27,26,25,99, 30,29,28,99,  // NOLINT
       3, 2, 1,99,  6, 5, 4,99,  9, 8, 7,99, 12,11,10,99, 15,14,13,99   // NOLINT
  ));                                                                   // NOLINT
  // clang-format on

  // Copy it again, with swizzling and dropping alpha.
  std::array<uint8_t, source_size> source_again;
  CopyAndFlipRaw<uint8_t, /* source_channels = */ dest_channels,
                 /* swizzle = */ true, /* dest_channels = */ source_channels>(
      dest.data(), source_again.data(), width, height);
  // clang-format off
  EXPECT_THAT(source_again, testing::ElementsAre(
       1, 2, 3,  4, 5, 6,  7, 8, 9, 10,11,12, 13,14,15,  // NOLINT
      16,17,18, 19,20,21, 22,23,24, 25,26,27, 28,29,30   // NOLINT
  ));                                                    // NOLINT
  // clang-format on
}

}  // namespace
}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
