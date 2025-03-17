#include "drake/visualization/colorize_depth_image.h"

#include <array>
#include <optional>

namespace drake {
namespace visualization {

using geometry::Rgba;
using systems::Context;
using systems::sensors::ImageDepth16U;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageRgba8U;

template <typename T>
ColorizeDepthImage<T>::ColorizeDepthImage() {
  this->DeclareAbstractInputPort("depth_image_32f", Value<ImageDepth32F>());
  this->DeclareAbstractInputPort("depth_image_16u", Value<ImageDepth16U>());
  this->DeclareAbstractOutputPort("color_image",
                                  &ColorizeDepthImage::CalcOutput);
}

template <typename T>
ColorizeDepthImage<T>::~ColorizeDepthImage() = default;

template <typename T>
void ColorizeDepthImage<T>::Calc(const ImageDepth32F& input,
                                 ImageRgba8U* output) const {
  DRAKE_DEMAND(output != nullptr);
  if (output->width() != input.width() || output->height() != input.height()) {
    output->resize(input.width(), input.height());
  }

  // Find the min & max pixel values, excluding too-near and too-far values.
  const auto is_valid = [](float pixel) {
    return (pixel > 0.0) && std::isfinite(pixel);
  };
  std::optional<double> min_pixel;
  std::optional<double> max_pixel;
  for (int i = 0; i < input.size(); ++i) {
    const float pixel = input.at(0, 0)[i];
    if (is_valid(pixel)) {
      if (!min_pixel || pixel < *min_pixel) {
        min_pixel = pixel;
      }
      if (!max_pixel || pixel > *max_pixel) {
        max_pixel = pixel;
      }
    }
  }
  if (!min_pixel.has_value()) {
    min_pixel = 0;
    max_pixel = 0;
  } else {
    DRAKE_DEMAND(max_pixel.has_value());
  }

  // Convert the invalid color to bytes.
  const std::array<uint8_t, 4> invalid = {
      static_cast<uint8_t>(invalid_color_.r() * 255),
      static_cast<uint8_t>(invalid_color_.g() * 255),
      static_cast<uint8_t>(invalid_color_.b() * 255),
      static_cast<uint8_t>(invalid_color_.a() * 255),
  };

  // Convert the depths to grayscale.
  const double depth_scale = (*min_pixel == *max_pixel)
                                 ? 1.0  // Avoid divide-by-zero.
                                 : (1.0 / (*max_pixel - *min_pixel));
  for (int v = 0; v < output->height(); ++v) {
    for (int u = 0; u < output->width(); ++u) {
      const float pixel = input.at(u, v)[0];
      if (is_valid(pixel)) {
        const double normalized_depth = (pixel - *min_pixel) * depth_scale;
        const uint8_t byte = 255 - static_cast<uint8_t>(normalized_depth * 255);
        for (int ch = 0; ch < 3; ++ch) {
          output->at(u, v)[ch] = byte;
        }
        output->at(u, v)[3] = 255;
      } else {
        for (int ch = 0; ch < 4; ++ch) {
          output->at(u, v)[ch] = invalid[ch];
        }
      }
    }
  }
}

template <typename T>
void ColorizeDepthImage<T>::Calc(const ImageDepth16U& input,
                                 ImageRgba8U* output) const {
  ImageDepth32F temp;
  ConvertDepth16UTo32F(input, &temp);
  Calc(temp, output);
}

template <typename T>
void ColorizeDepthImage<T>::CalcOutput(const Context<T>& context,
                                       ImageRgba8U* output) const {
  // Only one of the input ports should have value (not both or neither).
  const bool has_depth32f = this->get_input_port(0).HasValue(context);
  const bool has_depth16u = this->get_input_port(1).HasValue(context);
  DRAKE_THROW_UNLESS(has_depth32f != has_depth16u);
  if (has_depth32f) {
    const auto& depth =
        this->get_input_port(0).template Eval<ImageDepth32F>(context);
    Calc(depth, output);
  } else {
    const auto& depth =
        this->get_input_port(1).template Eval<ImageDepth16U>(context);
    Calc(depth, output);
  }
}

template class ColorizeDepthImage<double>;

}  // namespace visualization
}  // namespace drake
