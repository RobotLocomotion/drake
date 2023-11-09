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

}  // namespace sensors
}  // namespace systems
}  // namespace drake
