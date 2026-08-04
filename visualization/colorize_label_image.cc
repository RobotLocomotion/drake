#include "drake/visualization/colorize_label_image.h"

#include "drake/geometry/render/colorize_image.h"

namespace drake {
namespace visualization {

using systems::Context;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

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
  geometry::render::ColorizeLabelImage(input, output, background_color_);
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
