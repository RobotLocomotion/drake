#include "drake/geometry/render/colorize_image.h"

#include <array>
#include <cstdint>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/never_destroyed.h"
#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {
namespace render {

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

void ColorizeLabelImage(const systems::sensors::ImageLabel16I& input,
                        systems::sensors::ImageRgba8U* output,
                        const Rgba& background_color) {
  DRAKE_THROW_UNLESS(output != nullptr);
  if (output->width() != input.width() || output->height() != input.height()) {
    output->resize(input.width(), input.height());
  }
  const std::array<uint8_t, 4> background = {
      static_cast<uint8_t>(background_color.r() * 255),
      static_cast<uint8_t>(background_color.g() * 255),
      static_cast<uint8_t>(background_color.b() * 255),
      static_cast<uint8_t>(background_color.a() * 255),
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

}  // namespace render
}  // namespace geometry
}  // namespace drake
