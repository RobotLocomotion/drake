#include "drake/visualization/colorize_label_image.h"

#include <array>
#include <vector>

#include "drake/common/never_destroyed.h"
#include "drake/geometry/render/render_label.h"

namespace drake {
namespace visualization {

using geometry::Rgba;
using geometry::render::RenderLabel;
using systems::Context;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

namespace {
const std::vector<std::array<uint8_t, 4>>& GetDefaultPalette() {
  using Color = std::array<uint8_t, 4>;
  using Result = std::vector<Color>;
  static const never_destroyed<Result> result{Result{
      // Based on the TABLEAU_COLORS from matplotlib.
      Color{0x1f, 0x77, 0xb4, 0xff},  // blue
      Color{0xff, 0x7f, 0x0e, 0xff},  // orange
      Color{0x2c, 0xa0, 0x2c, 0xff},  // green
      Color{0xd6, 0x27, 0x28, 0xff},  // red
      Color{0x94, 0x67, 0xbd, 0xff},  // purple
      Color{0x8c, 0x56, 0x4b, 0xff},  // brown
      Color{0xe3, 0x77, 0xc2, 0xff},  // pink
      Color{0x7f, 0x7f, 0x7f, 0xff},  // gray
      Color{0xbc, 0xbd, 0x22, 0xff},  // olive
      Color{0x17, 0xbe, 0xcf, 0xff},  // cyan
  }};
  return result.access();
}
}  // namespace

template <typename T>
ColorizeLabelImage<T>::ColorizeLabelImage() {
  this->DeclareAbstractInputPort("label_image", Value(ImageLabel16I()));
  this->DeclareAbstractOutputPort("color_image",
                                  &ColorizeLabelImage::CalcOutput);
}

template <typename T>
ColorizeLabelImage<T>::~ColorizeLabelImage() = default;

template <typename T>
void ColorizeLabelImage<T>::Calc(const ImageLabel16I& input,
                                 ImageRgba8U* output) const {
  DRAKE_THROW_UNLESS(output != nullptr);
  if (output->width() != input.width() || output->height() != input.height()) {
    output->resize(input.width(), input.height());
  }
  const std::array<uint8_t, 4> background = {
      static_cast<uint8_t>(background_color_.r() * 255),
      static_cast<uint8_t>(background_color_.g() * 255),
      static_cast<uint8_t>(background_color_.b() * 255),
      static_cast<uint8_t>(background_color_.a() * 255),
  };
  const std::vector<std::array<uint8_t, 4>>& palette = GetDefaultPalette();
  for (int v = 0; v < output->height(); ++v) {
    for (int u = 0; u < output->width(); ++u) {
      const int16_t label = input.at(u, v)[0];
      const bool is_reserved = label > RenderLabel::kMaxUnreserved;
      const std::array<uint8_t, 4>& color =
          is_reserved ? background : palette[label % palette.size()];
      for (int ch = 0; ch < 4; ++ch) {
        output->at(u, v)[ch] = color[ch];
      }
    }
  }
}

template <typename T>
void ColorizeLabelImage<T>::CalcOutput(const Context<T>& context,
                                       ImageRgba8U* output) const {
  const auto& input =
      this->get_input_port().template Eval<ImageLabel16I>(context);
  Calc(input, output);
}

template class ColorizeLabelImage<double>;

}  // namespace visualization
}  // namespace drake
