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
  // Gather the the input images and their dimensions.
  const ImageRgba8U empty;
  MatrixX<const ImageRgba8U*> images(rows_, cols_);
  MatrixX<int> cell_widths(rows_, cols_);
  MatrixX<int> cell_heights(rows_, cols_);
  for (int row = 0; row < rows_; ++row) {
    for (int col = 0; col < cols_; ++col) {
      const InputPort<T>& input = *inputs_(row, col);
      if (input.HasValue(context)) {
        images(row, col) = &input.template Eval<ImageRgba8U>(context);
        cell_widths(row, col) = images(row, col)->width();
        cell_heights(row, col) = images(row, col)->height();
      } else {
        images(row, col) = &empty;
        cell_widths(row, col) = 0;
        cell_heights(row, col) = 0;
      }
    }
  }

  // Compute the tile layout. The height of each row (width of each column) will
  // be the largest value within that individual row (column).
  for (int row = 0; row < rows_; ++row) {
    cell_heights.row(row).array() = cell_heights.row(row).maxCoeff();
  }
  for (int col = 0; col < cols_; ++col) {
    cell_widths.col(col).array() = cell_widths.col(col).maxCoeff();
  }
  const int overall_width = cell_widths.row(0).sum();
  const int overall_height = cell_heights.col(0).sum();

  // Zero the output.
  output->resize(overall_width, overall_height);

  // Copy each input image into the output at the appropriate offset location.
  constexpr int kNumChannels = ImageRgba8U::kNumChannels;
  for (int row = 0; row < rows_; ++row) {
    for (int col = 0; col < cols_; ++col) {
      const ImageRgba8U& image = *images(row, col);
      const int u_offset = cell_widths.row(0).head(col).sum();
      const int v_offset = cell_heights.col(0).head(row).sum();
      for (int v = 0; v < image.height(); ++v) {
        for (int u = 0; u < image.width(); ++u) {
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
