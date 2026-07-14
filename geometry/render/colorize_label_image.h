#pragma once

#include "drake/geometry/rgba.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render {

/** Colorizes a label image into a color image using a fixed palette.
 Non-reserved labels (user-assigned values) are mapped to colors from a
 built-in palette. Reserved labels (empty, don't care, etc.) are mapped to
 `background_color`. */
void ColorizeLabelImage(const systems::sensors::ImageLabel16I& input,
                        systems::sensors::ImageRgba8U* output,
                        const Rgba& background_color = Rgba{0, 0, 0, 0});

}  // namespace render
}  // namespace geometry
}  // namespace drake
