#include <numeric>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkImageData.h>    // vtkCommonDataModel
#include <vtkImageExport.h>  // vtkIOImage

#include "drake/common/eigen_types.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/sensors/vtk_image_reader_writer.h"

namespace drake {
namespace systems {
namespace sensors {
namespace internal {
namespace {

namespace fs = std::filesystem;

// TestParams provides a grouping for value-parameterized test params.
struct TestParams {
  ImageFileFormat format{ImageFileFormat::kJpeg};

  // Whether to use files or memory buffers.
  bool write_to_file{false};

  // Whether to use uint16_t (instead of the natural type).
  bool force_16u{false};

  friend std::ostream& operator<<(std::ostream& os, const TestParams& self) {
    os << fmt::format("[format = {}; write_to_file = {}; force_16u = {}]",
                      self.format, self.write_to_file, self.force_16u);
    return os;
  }
};

class VtkImageReaderWriterTest : public testing::TestWithParam<TestParams> {
 public:
  VtkImageReaderWriterTest() = default;
};

// Checks MakeWriter() and MakeReader() back-to-back. This provides a simple
// sanity check for these low-level helper functions.
TEST_P(VtkImageReaderWriterTest, RoundTrip) {
  const ImageFileFormat format = GetParam().format;
  const bool write_to_file = GetParam().write_to_file;
  const bool force_16u = GetParam().force_16u;

  // Create a sample image in memory. The image data will be filled with
  // incrementing numbers, starting from 1 for the first datum.
  const int width = 3;
  const int height = 2;
  const int depth = 1;
  const int channels = (format == ImageFileFormat::kTiff) ? 1 : 3;
  const int total_storage = width * height * depth * channels;
  const int vtk_scalar = force_16u                            ? VTK_TYPE_UINT16
                         : (format == ImageFileFormat::kTiff) ? VTK_TYPE_FLOAT32
                                                              : VTK_TYPE_UINT8;
  vtkNew<vtkImageData> image;
  image->SetDimensions(width, height, depth);
  image->AllocateScalars(vtk_scalar, channels);
  if (vtk_scalar == VTK_TYPE_UINT8) {
    auto* dest = reinterpret_cast<uint8_t*>(image->GetScalarPointer());
    std::iota(dest, dest + total_storage, uint8_t{0x01});
  } else if (vtk_scalar == VTK_TYPE_UINT16) {
    auto* dest = reinterpret_cast<uint16_t*>(image->GetScalarPointer());
    std::iota(dest, dest + total_storage, uint16_t{0x0001});
  } else {
    auto* dest = reinterpret_cast<float*>(image->GetScalarPointer());
    std::iota(dest, dest + total_storage, 1.0f);
  }

  // Check MakeWriter() construction.
  // Set it up to write either to `filename` xor `writer_output`.
  fs::path filename;
  std::vector<uint8_t> writer_output;
  vtkSmartPointer<vtkImageWriter> writer;
  if (write_to_file) {
    filename =
        fs::path(temp_directory()) / fmt::format("RoundTrip_{}.image", format);
    writer = MakeWriter(format, filename);
  } else {
    writer = MakeWriter(format, &writer_output);
  }
  ASSERT_TRUE(writer != nullptr);

  // Ask MakeWriter() to write out the image.
  writer->SetInputData(image.Get());
  writer->Write();

  // Check MakeReader().
  vtkSmartPointer<vtkImageReader2> reader;
  if (write_to_file) {
    reader = MakeReader(format, filename);
  } else {
    reader = MakeReader(format, writer_output.data(), writer_output.size());
  }
  ASSERT_TRUE(reader != nullptr);
  reader->Update();

  // Check the image metadata. We'll use ASSERT not EXPECT because momentarily
  // we'll also compare the image data buffers, which we don't want to do when
  // the sizes were wrong.
  vtkNew<vtkImageExport> loader;
  loader->SetInputConnection(reader->GetOutputPort(0));
  loader->Update();
  const int* const dims = loader->GetDataDimensions();
  ASSERT_EQ(dims[0], width);
  ASSERT_EQ(dims[1], height);
  ASSERT_EQ(dims[2], depth);
  ASSERT_EQ(loader->GetDataNumberOfScalarComponents(), channels);
  ASSERT_EQ(loader->GetDataScalarType(), vtk_scalar);

  // For JPEG images, we can't exactly compare the pixels because the lossy
  // compression algorithm will have adjusted them slightly.
  if (format == ImageFileFormat::kJpeg) {
    // Instead, we'll check that the data is still monotonically increasing
    // (within some tolerance). This is sufficient to notice if any data was
    // mistakenly transposed.
    DRAKE_DEMAND(vtk_scalar == VTK_TYPE_UINT8);
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
}

// These abbreviations help make the test matrix below easier to read.
constexpr ImageFileFormat kJpeg = ImageFileFormat::kJpeg;
constexpr ImageFileFormat kPng = ImageFileFormat::kPng;
constexpr ImageFileFormat kTiff = ImageFileFormat::kTiff;

INSTANTIATE_TEST_SUITE_P(
    All, VtkImageReaderWriterTest,
    testing::Values(
        // VTK JPEG only supports 8-bit channels, so no 16u here.
        TestParams{.format = kJpeg, .write_to_file = true},
        TestParams{.format = kJpeg, .write_to_file = false},
        // VTK PNG supports both 8-bit and 16-bit channels.
        TestParams{.format = kPng, .write_to_file = true},
        TestParams{.format = kPng, .write_to_file = false},
        TestParams{.format = kPng, .write_to_file = true, .force_16u = true},
        TestParams{.format = kPng, .write_to_file = false, .force_16u = true},
        // VTK TIFF doesn't support writing to memory. We have a expect-throws
        // test immediately below to cover that.
        TestParams{.format = kTiff, .write_to_file = true},
        TestParams{.format = kTiff, .write_to_file = true, .force_16u = true}));

// Checks that an unsupported operation produces an error.
GTEST_TEST(UnparameterizedVtkImageReaderWriterTest, TiffWriteToMemoryThrow) {
  std::vector<uint8_t> output;
  DRAKE_EXPECT_THROWS_MESSAGE(MakeWriter(kTiff, &output),
                              ".*TIFF.*memory buffer.*");
}

}  // namespace
}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
