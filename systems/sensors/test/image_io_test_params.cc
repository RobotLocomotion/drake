#include "drake/systems/sensors/test/image_io_test_params.h"

#include <numeric>

namespace drake {
namespace systems {
namespace sensors {

vtkNew<vtkImageData> ImageIoTestParams::CreateIotaVtkImage() const {
  vtkNew<vtkImageData> image;
  image->SetDimensions(width(), height(), depth());
  image->AllocateScalars(vtk_scalar(), channels());
  if (vtk_scalar() == VTK_TYPE_UINT8) {
    auto* dest = reinterpret_cast<uint8_t*>(image->GetScalarPointer());
    std::iota(dest, dest + total_storage(), uint8_t{0x01});
  } else if (vtk_scalar() == VTK_TYPE_UINT16) {
    auto* dest = reinterpret_cast<uint16_t*>(image->GetScalarPointer());
    std::iota(dest, dest + total_storage(), uint16_t{0x0001});
  } else {
    auto* dest = reinterpret_cast<float*>(image->GetScalarPointer());
    std::iota(dest, dest + total_storage(), 1.0f);
  }
  return image;
}

ImageAny ImageIoTestParams::CreateIotaDrakeImage() const {
  if (force_gray && pixel_scalar() == PixelScalar::k8U) {
    DRAKE_DEMAND(channels() == 1);
    ImageGrey8U result(width(), height());
    auto* dest = result.at(0, 0);
    std::iota(dest, dest + result.size(), uint8_t{0x01});
    return result;
  }
  if (alpha && pixel_scalar() == PixelScalar::k8U) {
    DRAKE_DEMAND(channels() == 4);
    ImageRgba8U result(width(), height());
    auto* dest = result.at(0, 0);
    std::iota(dest, dest + result.size(), uint8_t{0x01});
    return result;
  }
  if (pixel_scalar() == PixelScalar::k8U) {
    DRAKE_DEMAND(channels() == 3);
    ImageRgb8U result(width(), height());
    auto* dest = result.at(0, 0);
    std::iota(dest, dest + result.size(), uint8_t{0x01});
    return result;
  }
  if (pixel_scalar() == PixelScalar::k16U) {
    DRAKE_DEMAND(channels() == 1);
    ImageDepth16U result(width(), height());
    auto* dest = result.at(0, 0);
    std::iota(dest, dest + result.size(), uint16_t{0x0001});
    return result;
  }
  if (pixel_scalar() == PixelScalar::k32F) {
    DRAKE_DEMAND(channels() == 1);
    ImageDepth32F result(width(), height());
    auto* dest = result.at(0, 0);
    std::iota(dest, dest + result.size(), 1.0f);
    return result;
  }
  throw std::logic_error("CreateIotaDrakeImage unsupported params");
}

void ImageIoTestParams::CompareImageBuffer(const void* original,
                                           const void* readback) const {
  // For JPEG images, we can't exactly compare the pixels because the lossy
  // compression algorithm will have adjusted them slightly.
  if (format == ImageFileFormat::kJpeg) {
    // Instead, we'll check that the data is still monotonically increasing
    // (within some tolerance). This is sufficient to notice if any data was
    // mistakenly transposed.
    DRAKE_DEMAND(pixel_scalar() == PixelScalar::k8U);
    auto* loaded = reinterpret_cast<const uint8_t*>(readback);
    Eigen::Map<const MatrixX<uint8_t>> head(loaded, 1, total_storage() - 1);
    Eigen::Map<const MatrixX<uint8_t>> tail(loaded + 1, 1, total_storage() - 1);
    const Eigen::Array<int, 1, Eigen::Dynamic> increments =
        tail.cast<int>().array() - head.cast<int>().array();
    const int max_increment = 2;
    EXPECT_LE(increments.abs().maxCoeff(), max_increment)
        << fmt::to_string(fmt_eigen(increments.matrix()));
    return;
  }

  // Compare the loaded pixels to the original image. We'll compare using
  // an Eigen::Map wrapper over the image data, to get a better printout.
  if (pixel_scalar() == PixelScalar::k8U) {
    auto* orig = reinterpret_cast<const uint8_t*>(original);
    auto* loaded = reinterpret_cast<const uint8_t*>(readback);
    Eigen::Map<const MatrixX<uint8_t>> orig_map(orig, 1, total_storage());
    Eigen::Map<const MatrixX<uint8_t>> loaded_map(loaded, 1, total_storage());
    EXPECT_EQ(orig_map.template cast<int>(), loaded_map.template cast<int>());
  } else if (pixel_scalar() == PixelScalar::k16U) {
    auto* orig = reinterpret_cast<const uint16_t*>(original);
    auto* loaded = reinterpret_cast<const uint16_t*>(readback);
    Eigen::Map<const MatrixX<uint16_t>> orig_map(orig, 1, total_storage());
    Eigen::Map<const MatrixX<uint16_t>> loaded_map(loaded, 1, total_storage());
    EXPECT_EQ(orig_map, loaded_map);
  } else {
    DRAKE_DEMAND(pixel_scalar() == PixelScalar::k32F);
    auto* orig = reinterpret_cast<const float*>(original);
    auto* loaded = reinterpret_cast<const float*>(readback);
    Eigen::Map<const MatrixX<float>> orig_map(orig, 1, total_storage());
    Eigen::Map<const MatrixX<float>> loaded_map(loaded, 1, total_storage());
    EXPECT_EQ(orig_map, loaded_map);
  }
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
