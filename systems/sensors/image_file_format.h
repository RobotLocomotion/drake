#pragma once

#include <string>

#include "drake/common/fmt.h"

namespace drake {
namespace systems {
namespace sensors {

/** The image file formats known to Drake. */
enum class ImageFileFormat {
  /** mime-type: image/jpeg. */
  kJpeg,
  /** mime-type: image/png. */
  kPng,
  /** mime-type: image/tiff. */
  kTiff,
};

std::string to_string(ImageFileFormat);

}  // namespace sensors
}  // namespace systems
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::systems::sensors, ImageFileFormat, x,
                   drake::systems::sensors::to_string(x))
