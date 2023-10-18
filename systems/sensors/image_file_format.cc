#include "drake/systems/sensors/image_file_format.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace sensors {

std::string to_string(ImageFileFormat format) {
  switch (format) {
    case ImageFileFormat::kJpeg:
      return "jpeg";
    case ImageFileFormat::kPng:
      return "png";
    case ImageFileFormat::kTiff:
      return "tiff";
  }
  DRAKE_UNREACHABLE();
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
