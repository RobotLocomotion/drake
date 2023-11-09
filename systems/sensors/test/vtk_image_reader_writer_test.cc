#include <gmock/gmock.h>
#include <gtest/gtest.h>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkImageData.h>    // vtkCommonDataModel
#include <vtkImageExport.h>  // vtkIOImage

#include "drake/common/eigen_types.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/sensors/test/image_io_test_params.h"
#include "drake/systems/sensors/vtk_image_reader_writer.h"

namespace drake {
namespace systems {
namespace sensors {
namespace internal {
namespace {

namespace fs = std::filesystem;

class VtkImageReaderWriterTest
    : public testing::TestWithParam<ImageIoTestParams> {
 public:
  VtkImageReaderWriterTest() = default;
};

// Checks MakeWriter() and MakeReader() back-to-back. This provides a simple
// sanity check for these low-level helper functions.
TEST_P(VtkImageReaderWriterTest, RoundTrip) {
  const ImageIoTestParams& param = GetParam();
  const ImageFileFormat format = param.format;
  const bool write_to_file = param.write_to_file;
  const int vtk_scalar = param.vtk_scalar();

  // Create a sample image in memory.
  auto image = param.CreateIotaVtkImage();
  const int total_storage = param.total_storage();

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
  ASSERT_EQ(dims[0], param.width());
  ASSERT_EQ(dims[1], param.height());
  ASSERT_EQ(dims[2], param.depth());
  ASSERT_EQ(loader->GetDataNumberOfScalarComponents(), param.channels());
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

INSTANTIATE_TEST_SUITE_P(All, VtkImageReaderWriterTest,
                         GetAllImageIoTestParams());

// Checks that an unsupported operation produces an error.
GTEST_TEST(UnparameterizedVtkImageReaderWriterTest, TiffWriteToMemoryThrow) {
  std::vector<uint8_t> output;
  DRAKE_EXPECT_THROWS_MESSAGE(MakeWriter(ImageFileFormat::kTiff, &output),
                              ".*TIFF.*memory buffer.*");
}

}  // namespace
}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
