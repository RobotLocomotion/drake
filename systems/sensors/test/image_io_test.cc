#include "drake/systems/sensors/image_io.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
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

class ImageIoFileFormatTest : public testing::TestWithParam<ImageIoTestParams> {
 public:
  ImageIoFileFormatTest() = default;
};

// Runs Save() and Load() back to back. This is provides a simple sanity check.
TEST_P(ImageIoFileFormatTest, RoundTrip) {
  const ImageIoTestParams& param = GetParam();
  const ImageFileFormat format = param.format;
  const bool write_to_file = param.write_to_file;

  // XXX
  return;

  // Create a sample image.
  const ImageAny original = param.CreateIotaDrakeImage();

  // Call Save() then Load() back to back.
  ImageAny readback;
  if (write_to_file) {
    fs::path filename =
        fs::path(temp_directory()) / fmt::format("RoundTrip_{}.image", format);
    std::visit(
        [&](const auto& image) {
          ImageIo(format).Save(image, filename);
        },
        original);
    readback = ImageIo(format).Load(filename);
  } else {
    const std::vector<uint8_t> buffer = std::visit(
        [&](const auto& image) {
          return ImageIo(format).Save(image);
        },
        original);
    const ImageIo::ByteSpan span{buffer.data(), buffer.size()};
    readback = ImageIo(format).Load(span);
  }

#if 0
  // For JPEG images, we can't exactly compare the pixels because the lossy
  // compression algorithm will have adjusted them slightly.
  if (format == ImageFileFormat::kJpeg) {
    // Instead, we'll check that the data is still monotonically increasing
    // (within some tolerance). This is sufficient to notice if any data was
    // mistakenly transposed.
    DRAKE_DEMAND(param.vtk_scalar == VTK_TYPE_UINT8);
    auto* loaded = reinterpret_cast<const uint8_t*>(loader->GetPointerToData());
    Eigen::Map<const MatrixX<uint8_t>> head(loaded, 1, total_storage - 1);
    Eigen::Map<const MatrixX<uint8_t>> tail(loaded + 1, 1, total_storage - 1);
    const Eigen::Array<int, 1, Eigen::Dynamic> increments =
        tail.cast<int>().array() - head.cast<int>().array();
    const int max_increment = 2;
    EXPECT_LE(increments.abs().maxCoeff(), max_increment)
        << fmt::to_string(fmt_eigen(increments.matrix()));
    return;
  }
#endif

#if 0
  // Compare the loaded pixels to the original image. We'll compare using
  // an Eigen::Map wrapper over the image data, to get a better printout.
  if (vtk_scalar == VTK_TYPE_UINT8) {
    auto* orig = reinterpret_cast<const uint8_t*>(image->GetScalarPointer());
    auto* loaded = reinterpret_cast<const uint8_t*>(loader->GetPointerToData());
    Eigen::Map<const MatrixX<uint8_t>> orig_map(orig, 1, total_storage);
    Eigen::Map<const MatrixX<uint8_t>> loaded_map(loaded, 1, total_storage);
    EXPECT_EQ(orig_map.template cast<int>(), loaded_map.template cast<int>());
  } else if (vtk_scalar == VTK_TYPE_UINT16) {
    auto* orig = reinterpret_cast<const uint16_t*>(image->GetScalarPointer());
    auto* loaded =
        reinterpret_cast<const uint16_t*>(loader->GetPointerToData());
    Eigen::Map<const MatrixX<uint16_t>> orig_map(orig, 1, total_storage);
    Eigen::Map<const MatrixX<uint16_t>> loaded_map(loaded, 1, total_storage);
    EXPECT_EQ(orig_map, loaded_map);
  } else {
    auto* orig = reinterpret_cast<const float*>(image->GetScalarPointer());
    auto* loaded = reinterpret_cast<const float*>(loader->GetPointerToData());
    Eigen::Map<const MatrixX<float>> orig_map(orig, 1, total_storage);
    Eigen::Map<const MatrixX<float>> loaded_map(loaded, 1, total_storage);
    EXPECT_EQ(orig_map, loaded_map);
  }
#endif
}

INSTANTIATE_TEST_SUITE_P(All, ImageIoFileFormatTest, GetAllImageIoTestParams());

// XXX
GTEST_TEST(ImageIoInternalTest, DISABLED_TODO) {
  // XXX
}

}  // namespace
}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
