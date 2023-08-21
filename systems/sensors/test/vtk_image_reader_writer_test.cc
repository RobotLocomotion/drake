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

// ReaderWriterParams provides a grouping for value-parameterized test params.
struct ReaderWriterParams {
  ImageFileFormat format{};

  // Whether to use files or memory buffers.
  bool write_to_file{};

  // Whether to use a wide scalar type (uint16_t instead of uint8_t).
  bool wide{};
};

class VtkImageReaderWriterParamsTest
    : public testing::TestWithParam<ReaderWriterParams> {
 public:
  VtkImageReaderWriterParamsTest() = default;
};
// Checks MakeWriter() and MakeReader() back-to-back. This provides a simple
// sanity check for these low-level helper functions.
TEST_P(VtkImageReaderWriterParamsTest, RoundTrip) {
  const ImageFileFormat format = GetParam().format;
  const bool write_to_file = GetParam().write_to_file;
  const bool wide = GetParam().wide;

  // Create a sample image in memory. The image data will be filled with
  // incrementing numbers, starting from 1 for the first datum.
  const int width = 3;
  const int height = 2;
  const int depth = 1;
  const int channels = (format == ImageFileFormat::kTiff) ? 1 : 3;
  const int total_storage = width * height * depth * channels;
  const int vtk_scalar = (format == ImageFileFormat::kTiff) ? VTK_TYPE_FLOAT32
                         : wide                             ? VTK_TYPE_UINT16
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

  // Check GetMetadata(). Use ASSERT not EXPECT because momentarily we'll also
  // check the image pixels, which we don't want to do if the sizes were wrong.
  vtkNew<vtkImageExport> loader;
  loader->SetInputConnection(reader->GetOutputPort(0));
  loader->Update();

  // For JPEG images, we can't exactly compare the pixels because the lossy
  // compression algorithm will have adjusted them slightly.
  if (format == ImageFileFormat::kJpeg) {
    // TODO(jwnimmer-tri) Find better test coverage for JPEG contents.
    return;
  }

  // Compare the loaded pixels to the original image.
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

INSTANTIATE_TEST_SUITE_P(
    All, VtkImageReaderWriterParamsTest,
    testing::Values(
        ReaderWriterParams{ImageFileFormat::kJpeg, .write_to_file = true},
        ReaderWriterParams{ImageFileFormat::kJpeg, .write_to_file = false},
        ReaderWriterParams{ImageFileFormat::kPng, .write_to_file = true},
        ReaderWriterParams{ImageFileFormat::kPng, .write_to_file = false},
        ReaderWriterParams{ImageFileFormat::kPng, .write_to_file = true,
                           .wide = true},
        ReaderWriterParams{ImageFileFormat::kPng, .write_to_file = false,
                           .wide = true},
        // NOLINTNEXTLINE
        ReaderWriterParams{ImageFileFormat::kTiff, .write_to_file = true}
        // No "false" case here; VTK TIFF doesn't support writing to memory.
        // We have a expect-throws test immediate below for this case.
        // NOLINTNEXTLINE
        ));

// Checks that an unsupported operation produces an error.
GTEST_TEST(VtkImageReaderWriterTest, TiffWriteToMemoryThrow) {
  std::vector<uint8_t> output;
  DRAKE_EXPECT_THROWS_MESSAGE(MakeWriter(ImageFileFormat::kTiff, &output),
                              ".*TIFF.*memory buffer.*");
}

}  // namespace
}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
