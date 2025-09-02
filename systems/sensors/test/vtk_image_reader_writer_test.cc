#include <vector>

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

  const void* const original = image->GetScalarPointer();
  const void* const readback = loader->GetPointerToData();
  param.CompareImageBuffer(original, readback);
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
