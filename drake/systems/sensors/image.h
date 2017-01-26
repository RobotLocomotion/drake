#pragma once

#include <vector>
#include <utility>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace sensors {

/// Simple data format for Image that takes an arbitrary type for the each
/// channel in a pixel.  For the complex calculation with the image, consider
/// converting this to other libaries' Matrix data format, i.e., MatrixX in
/// Eigen, Mat in OpenCV, and so on.
///
/// The origin of image coordinate system is on the left-upper corner.
///
/// @ingroup sensor_systems
template <typename T>
class Image {
 public:
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

  /// Default copy constructor
  Image(const Image<T>&) = default;

  /// Default assignment operator
  Image& operator=(const Image<T>&) = default;

  /// Move constructor.  After the move, the sizes of the width and height and
  /// the number of channels for the source object become zero.
  Image(Image<T>&& other) : width_(other.width_),
                            height_(other.height_),
                            channel_(other.channel_),
                            data_(std::move(other.data_)) {
    other.width_ = 0;
    other.height_ = 0;
    other.channel_ = 0;
  }

  /// Move assignment operator.  After the move, the sizes of the width and
  /// height and the number of channels for the source object become zero.
  Image& operator=(Image<T>&& other) {
    if (this != &other) {
      width_ = other.width_;
      height_ = other.height_;
      channel_ = other.channel_;
      data_ = std::move(other.data_);

      other.width_ = 0;
      other.height_ = 0;
      other.channel_ = 0;
    }
    return *this;
  }

  /// Return the size of width for the image
  int width() const { return width_; }

  /// Return the size of height for the image
  int height() const { return height_; }

  /// Return the number of channel for the image
  int num_channels() const { return channel_; }

  /// Return the result of the number of pixels in a image by the number of
  /// channels in a pixel
  int size() const { return width_ * height_ * channel_; }

  /// Change the sizes of the width and height for the image.  The values for
  /// them should be greater than zero.  All the values in the pixels become
  /// zero after resize.
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
  int width_;
  int height_;
  int channel_;
  std::vector<T> data_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
