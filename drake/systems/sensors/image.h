#pragma once

#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/reinit_after_move.h"

namespace drake {

namespace systems {
namespace sensors {

/// Simple data format for Image that takes an arbitrary type for the each
/// channel in a pixel.  For the complex calculation with the image, consider
/// converting this to other libaries' Matrix data format, i.e., MatrixX in
/// Eigen, Mat in OpenCV, and so on.
///
/// The origin of image coordinate system is on the left-upper corner.
template <typename T>
class Image {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Image)

  /// Image size and number of channel only constructor.  Specifies a width,
  /// height and number of channels for the image.  All the channel values in
  /// all the pixels are initialized with zero.
  /// @param width Size of width for image which should be greater than zero
  /// @param height Size of height for image which should be greater than zero
  /// @param channel Number of channels that a pixel contains. This should be
  /// greater than zero
  Image(int width, int height, int channel)
      : Image(width, height, channel, 0) {}

  /// Image size, number of channel and initial value constructor.  Specifies a
  /// width, height and number of channels for the image and an initial value
  /// for all the channels in all the pixels.
  /// @param width Size of width for image which should be greater than zero.
  /// @param height Size of height for image which should be greater than zero.
  /// @param channel Number of channels that a pixel contains. This should be
  /// greater than zero
  /// @param initial_value A value set to all the channels in all the pixels
  Image(int width, int height, int channel, T initial_value)
      : width_(width), height_(height), channel_(channel),
        data_(width * height * channel, initial_value) {
    DRAKE_ASSERT(width > 0);
    DRAKE_ASSERT(height > 0);
    DRAKE_ASSERT(channel > 0);
  }

  /// Constructs a zero-sized image with zero channels.
  Image() = default;

  /// Returns the size of width for the image
  int width() const { return width_; }

  /// Returns the size of height for the image
  int height() const { return height_; }

  /// Returns the number of channel for the image
  int num_channels() const { return channel_; }

  /// Returns the result of the number of pixels in a image by the number of
  /// channels in a pixel
  int size() const { return width_ * height_ * channel_; }

  /// Changes the sizes of the width and height for the image.  The values for
  /// them should be greater than zero.  (To resize to zero, assign a default-
  /// constructed Image into this; do not use this method.)  All the values in
  /// the pixels become zero after resize.
  void resize(int width, int height) {
    DRAKE_ASSERT(width > 0);
    DRAKE_ASSERT(height > 0);

    data_.resize(width * height * channel_);
    std::fill(data_.begin(), data_.end(), 0);
    width_ = width;
    height_ = height;
  }

  /// Access to the pixel located at (x, y) in image coordinate system where x
  /// is the variable for horizontal direction and y is one for vertical
  /// direction.  To access to the each channel value in the pixel (x, y),
  /// you can do:
  ///
  /// Image<uint8_t> image(640, 480, 4, 255);
  /// uint8_t red   = image.at(x, y)[0];
  /// uint8_t green = image.at(x, y)[1];
  /// uint8_t blue  = image.at(x, y)[2];
  /// uint8_t alpha = image.at(x, y)[3];
  T* at(int x, int y) {
    DRAKE_ASSERT(x >= 0 && x < width_);
    DRAKE_ASSERT(y >= 0 && y < height_);
    return data_.data() + (x + y * width_) * channel_;
  }

  /// Const version of at() method.  See the document for the non-const version
  /// for the detail.
  const T* at(int x, int y) const {
    DRAKE_ASSERT(x >= 0 && x < width_);
    DRAKE_ASSERT(y >= 0 && y < height_);
    return data_.data() + (x + y * width_) * channel_;
  }

 private:
  reinit_after_move<int> width_;
  reinit_after_move<int> height_;
  reinit_after_move<int> channel_;
  std::vector<T> data_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
