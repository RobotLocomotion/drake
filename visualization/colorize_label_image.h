#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/geometry/rgba.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace visualization {

/** %ColorizeLabelImage converts a label image to a color image.

@system
name: ColorizeLabelImage
input_ports:
- label_image
output_ports:
- color_image
@endsystem

Labels are mapped to colors with a built-in, fixed palette. The palette has
fewer elements than all possible labels, so not all labels will necessarily be
represented by a unique color. For label pixels that do not represent a label
("don't care", "empty", "unspecified", etc.), the color pixel will use the
`background_color` property (by default, black with 0% alpha).

@tparam_double_only
@ingroup visualization */
template <typename T>
class ColorizeLabelImage final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ColorizeLabelImage);

  /** Creates a %ColorizeLabelImage system. */
  ColorizeLabelImage();

  ~ColorizeLabelImage() final;

  /** Gets the color used for pixels with no label. */
  geometry::Rgba get_background_color() const { return background_color_; }

  /** Sets the color used for pixels with no label. */
  void set_background_color(const geometry::Rgba& background_color) {
    background_color_ = background_color;
  }

  /** Colorizes the `input` into `output`, without using any System port
  conections nor any Context. */
  void Calc(const systems::sensors::ImageLabel16I& input,
            systems::sensors::ImageRgba8U* output) const;

 private:
  void CalcOutput(const systems::Context<T>& context,
                  systems::sensors::ImageRgba8U* output) const;

  geometry::Rgba background_color_{0, 0, 0, 0};
};

}  // namespace visualization
}  // namespace drake
