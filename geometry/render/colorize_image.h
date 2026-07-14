#pragma once

#include "drake/geometry/rgba.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render {

/** Colorizes an input label image into an output color image using a fixed
 palette. Non-reserved labels (user-assigned values) are mapped to colors from a
 built-in palette. Reserved labels (empty, don't care, etc.) are mapped to
 `background_color`.

 @param input             The input label image.
 @param[out] output       The image the colorized image will be written to. It
                          will be resized to match `input` as necessary and its
                          contents will be completely overwritten.
 @param background_color  The color to use for all reserved label values.
 @pre `output != nullptr`. */
void ColorizeLabelImage(const systems::sensors::ImageLabel16I& input,
                        systems::sensors::ImageRgba8U* output,
                        const Rgba& background_color = Rgba{0, 0, 0, 0});

}  // namespace render
}  // namespace geometry
}  // namespace drake
