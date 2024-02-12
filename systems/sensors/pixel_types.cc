#include "drake/systems/sensors/pixel_types.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace sensors {

std::string to_string(PixelType x) {
  switch (x) {
    case PixelType::kRgb8U:
      return "Rgb8U";
    case PixelType::kBgr8U:
      return "Bgr8U";
    case PixelType::kRgba8U:
      return "Rgba8U";
    case PixelType::kBgra8U:
      return "Bgra8U";
    case PixelType::kGrey8U:
      return "Grey8U";
    case PixelType::kDepth16U:
      return "Depth16U";
    case PixelType::kDepth32F:
      return "Depth32F";
    case PixelType::kLabel16I:
      return "Label16I";
  }
  DRAKE_UNREACHABLE();
}

std::string to_string(PixelFormat x) {
  switch (x) {
    case PixelFormat::kRgb:
      return "Rgb";
    case PixelFormat::kBgr:
      return "Bgr";
    case PixelFormat::kRgba:
      return "Rgba";
    case PixelFormat::kBgra:
      return "Bgra";
    case PixelFormat::kGrey:
      return "Grey";
    case PixelFormat::kDepth:
      return "Depth";
    case PixelFormat::kLabel:
      return "Label";
  }
  DRAKE_UNREACHABLE();
}

std::string to_string(PixelScalar x) {
  switch (x) {
    case PixelScalar::k8U:
      return "8U";
    case PixelScalar::k16I:
      return "16I";
    case PixelScalar::k16U:
      return "16U";
    case PixelScalar::k32F:
      return "32F";
  }
  DRAKE_UNREACHABLE();
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
