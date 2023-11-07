#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/geometry/rgba.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace visualization {

/** %ColorizeDepthImage converts a depth image, either 32F or 16U, to a color
image. One input port, and only one, must be connected.

@system
name: ColorizeDepthImage
input_ports:
- depth_image_32f
- depth_image_16u
output_ports:
- color_image
@endsystem

Depth measurements are linearly mapped to a grayscale palette, with smaller
(closer) values brighter and larger (further) values darker.

The dynamic range of each input image determines the scale. The pixel with the
smallest depth will be fully white (#FFFFFFFF), and largest depth will be fully
black (#000000FF). Note that alpha channel is still 100% in both cases.

Because the dynamic range is measured one input image a time, take note that a
video recording of this System will not have consistent scaling across its
entirety. The ability to set a fixed palette is future work.

For the special depth pixel values "too close" or "too far", the color pixel
will use the ``invalid_color`` property (by default, a dim red).

@tparam_double_only
@ingroup visualization */
template <typename T>
class ColorizeDepthImage final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ColorizeDepthImage);

  /** Creates a %ColorizeDepthImage system. */
  ColorizeDepthImage();

  ~ColorizeDepthImage() final;

  /** Gets the color used for pixels with too-near or too-far depth. */
  geometry::Rgba get_invalid_color() const { return invalid_color_; }

  /** Sets the color used for pixels with too-near or too-far depth. */
  void set_invalid_color(const geometry::Rgba& invalid_color) {
    invalid_color_ = invalid_color;
  }

  /** Colorizes the `input` into `output`, without using any System port
  conections nor any Context. */
  void Calc(const systems::sensors::ImageDepth32F& input,
            systems::sensors::ImageRgba8U* output) const;

  /** Colorizes the `input` into `output`, without using any System port
  conections nor any Context. */
  void Calc(const systems::sensors::ImageDepth16U& input,
            systems::sensors::ImageRgba8U* output) const;

 private:
  void CalcOutput(const systems::Context<T>& context,
                  systems::sensors::ImageRgba8U* output) const;

  geometry::Rgba invalid_color_{100.0 / 255.0, 0, 0, 1.0};
};

}  // namespace visualization
}  // namespace drake
