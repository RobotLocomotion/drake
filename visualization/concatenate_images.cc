#include "drake/visualization/concatenate_images.h"

namespace drake {
namespace visualization {

using systems::Context;
using systems::InputPort;
using systems::sensors::ImageRgba8U;

// TODO(jwnimmer-tri) Add an argument for pixel (image) type. For now, we only
// support ImageRgba8U.
template <typename T>
ConcatenateImages<T>::ConcatenateImages(int rows, int cols)
    : rows_(rows), cols_(cols) {
  DRAKE_THROW_UNLESS(rows >= 1);
  DRAKE_THROW_UNLESS(cols >= 1);
  inputs_.resize(rows, cols);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      inputs_(row, col) = &this->DeclareAbstractInputPort(
          fmt::format("color_image_r{}_c{}", row, col), Value(ImageRgba8U()));
    }
  }
  this->DeclareAbstractOutputPort("color_image",
                                  &ConcatenateImages::CalcOutput);
}

template <typename T>
ConcatenateImages<T>::~ConcatenateImages() = default;

template <typename T>
const InputPort<T>& ConcatenateImages<T>::get_input_port(int row,
                                                         int col) const {
  DRAKE_THROW_UNLESS(row >= 0);
  DRAKE_THROW_UNLESS(col >= 0);
  DRAKE_THROW_UNLESS(row < rows_);
  DRAKE_THROW_UNLESS(col < cols_);
  return *inputs_(row, col);
}

template <typename T>
void ConcatenateImages<T>::CalcOutput(const Context<T>& context,
                                      ImageRgba8U* output) const {
  constexpr int kNumChannels = ImageRgba8U::kNumChannels;
  const ImageRgba8U& cell = inputs_(0, 0)->template Eval<ImageRgba8U>(context);
  const int cell_width = cell.width();
  const int cell_height = cell.height();
  const int overall_width = cell_width * cols_;
  const int overall_height = cell_height * rows_;
  if (output->width() != overall_width || output->height() != overall_height) {
    output->resize(overall_width, overall_height);
  }
  for (int row = 0; row < rows_; ++row) {
    for (int col = 0; col < cols_; ++col) {
      const ImageRgba8U& image =
          inputs_(row, col)->template Eval<ImageRgba8U>(context);
      DRAKE_THROW_UNLESS(image.width() == cell_width);
      DRAKE_THROW_UNLESS(image.height() == cell_height);
      const int v_offset = row * cell_height;
      const int u_offset = col * cell_width;
      for (int v = 0; v < cell_height; ++v) {
        for (int u = 0; u < cell_width; ++u) {
          for (int ch = 0; ch < kNumChannels; ++ch) {
            output->at(u_offset + u, v_offset + v)[ch] = image.at(u, v)[ch];
          }
        }
      }
    }
  }
}

template class ConcatenateImages<double>;

}  // namespace visualization
}  // namespace drake
