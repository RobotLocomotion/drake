#pragma once

#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace sensors {

/// Simple data format for Image that takes an arbitrary number of channel and
/// type for the each channel in a pixel.  For the complex calculation with the
/// image, consider converting this to other libaries' Matrix data format, i.e.,
/// MatrixX in Eigen, Mat in OpenCV, and so on.
///
/// The origin of image coordinate system is on the left-upper corner.
///
/// @ingroup sensor_systems
template <typename T, int channel>
class Image {
 public:

  /// Image size only constructor.  Specifies a width and height for the
  /// image.  All the pixel values are initialized with zero.
  /// @param width Size of width for image which should be greater than zero
  /// @param height Size of height for image which should be greater than zero
  Image(int width, int height) : width_(width), height_(height),
                                 data_(width * height * channel, 0) {
    DRAKE_ASSERT(width > 0);
    DRAKE_ASSERT(height > 0);
    DRAKE_ASSERT(channel > 0);
  }

  /// Image size and initial value constructor.  Specifies a width and height
  /// for the image and an initial value for all the channels in all the pixels.
  /// @param width Size of width for image which should be greater than zero.
  /// @param height Size of height for image which should be greater than zero.
  /// @param initial_value A value set to all the channels in all the pixels
  Image(int width, int height, T initial_value)
      : width_(width), height_(height),
        data_(width * height * channel, initial_value) {
    DRAKE_ASSERT(width > 0);
    DRAKE_ASSERT(height > 0);
    DRAKE_ASSERT(channel > 0);
  }

  /// Default copy constructor
  Image(const Image<T, channel>&) = default;

  /// Default assignment operator
  Image& operator=(const Image<T, channel>&) = default;

  /// Move constructor.  After the move, the sizes of the width and height for
  /// the source object become zero.
  Image(Image<T, channel>&& other) : width_(other.width_),
                                     height_(other.height_),
                                     data_(std::move(other.data_)) {
    other.width_ = 0;
    other.height_ = 0;
  }

  /// Move assignment operator.  After the move, the sizes of the width and
  /// height for the source object become zero.
  Image& operator=(Image<T, channel>&& other) {
    if (this != &other) {
      width_ = other.width_;
      height_ = other.height_;
      data_ = std::move(other.data_);

      other.width_ = 0;
      other.height_ = 0;
    }
    return *this;
  }

  /// Return the size of width for the image
  int width() const { return width_; }

  /// Return the size of height for the image
  int height() const { return height_; }

  /// Return the number of channel for the image
  int num_channels() const { return channel; }

  /// Return the result of the number of pixels in a image by the number of
  /// channels in a pixel
  int size() const { return width_ * height_ * channel; }

  /// Change the sizes of the width and height for the image.  The values for
  /// them should be greater than zero.
  void resize(int width, int height) {
    DRAKE_ASSERT(width > 0);
    DRAKE_ASSERT(height > 0);

    data_.resize(width * height * channel);
    width_ = width;
    height_= height;
  }

  /// Access to the pixel located at (x, y) in image coordinate system where x
  /// is the variable for horizontal direction and y is one for vertical
  /// direction.  To access to the each channel value in the pixel (x, y),
  /// you can do:
  ///
  /// Image<uint8_t, 4> image(640, 480, 255);
  /// uint8_t red   = image.at(x, y)[0];
  /// uint8_t green = image.at(x, y)[1];
  /// uint8_t blue  = image.at(x, y)[2];
  /// uint8_t alpha = image.at(x, y)[3];
  T* at(int x, int y) {
    DRAKE_ASSERT(x >= 0 && x < width_);
    DRAKE_ASSERT(y >= 0 && y < height_);
    return data_.data() + (x + y * width_) * channel;
  }

  /// Const version of at() method.  See the document for the non-const version
  /// for the detail.
  const T* at(int x, int y) const {
    DRAKE_ASSERT(x >= 0 && x < width_);
    DRAKE_ASSERT(y >= 0 && y < height_);
    return data_.data() + (x + y * width_) * channel;
  }

 private:
  int width_;
  int height_;
  std::vector<T> data_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
