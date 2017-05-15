#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/reinit_after_move.h"

namespace drake {

namespace systems {
namespace sensors {

/// The enum class to be used for describing pixel format in Image class.
/// The naming rule for the enum members is:
/// k + (semantic meaning of pixel) + (bit per channel) + (type per channel).
/// For the type per channel, one of the following capital letters is used.
///   - I: int
///   - U: unsigned int
///   - F: float
enum class PixelFormat {
  /// The pixel format used by ImageRgb8U.
  kRgb8U = 0,
  /// The pixel format used by ImageBgr8U.
  kBgr8U,
  /// The pixel format used by ImageRgba8U.
  kRgba8U,
  /// The pixel format used by ImageBgra8U.
  kBgra8U,
  /// The pixel format used by ImageGrey8U.
  kGrey8U,
  /// The pixel format used by ImageDepth16U.
  kDepth16U,
  /// The pixel format used by ImageDepth32F.
  kDepth32F,
  /// The pixel format used by ImageLabel16I.
  kLabel16I,
};

// Forward declaration of Image.
template <typename T, int channel, PixelFormat pixelformat> class Image;

/// The type for RGB image where the each channel has the type of uint8_t.
typedef Image<uint8_t, 3, PixelFormat::kRgb8U> ImageRgb8U;

/// The type for BGR image where the each channel has the type of uint8_t.
typedef Image<uint8_t, 3, PixelFormat::kBgr8U> ImageBgr8U;

/// The type for RGBA image where the each channel has the type of uint8_t.
typedef Image<uint8_t, 4, PixelFormat::kRgba8U> ImageRgba8U;

/// The type for BGRA image where the each channel has the type of uint8_t.
typedef Image<uint8_t, 4, PixelFormat::kBgra8U> ImageBgra8U;

/// The type for greyscale image where the channel has the type of uint8_t.
typedef Image<uint8_t, 1, PixelFormat::kGrey8U> ImageGrey8U;

/// The type for depth image where the channel has the type of uint16_t.
typedef Image<uint16_t, 1, PixelFormat::kDepth16U> ImageDepth16U;

/// The type for depth image where the channel has the type of float.
typedef Image<float, 1, PixelFormat::kDepth32F> ImageDepth32F;

/// The type for label image where the channel has the type of int16_t.
typedef Image<int16_t, 1, PixelFormat::kLabel16I> ImageLabel16I;


/// Simple data format for Image. For the complex calculation with the image,
/// consider converting this to other libaries' Matrix data format, i.e.,
/// MatrixX in Eigen, Mat in OpenCV, and so on.
///
/// The origin of image coordinate system is on the left-upper corner.
///
/// @tparam T The type of the each channel in a pixel.
/// @tparam channel The number of channels in a piexl.
/// @tparam pixelformat The pixel format in a pixel.
template <typename T, int channel, PixelFormat pixelformat>
class Image {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Image)

  /// Image size only constructor.  Specifies a width and height for the image.
  /// All the channel values in all the pixels are initialized with zero.
  /// @param width Size of width for image which should be greater than zero
  /// @param height Size of height for image which should be greater than zero
  Image(int width, int height)
      : Image(width, height, 0) {}

  /// Image size and initial value constructor.  Specifies a
  /// width, height and an initial value for all the channels in all the pixels.
  /// @param width Size of width for image which should be greater than zero.
  /// @param height Size of height for image which should be greater than zero.
  /// @param initial_value A value set to all the channels in all the pixels
  Image(int width, int height, T initial_value)
      : width_(width), height_(height),
        data_(width * height * channel, initial_value) {
    DRAKE_ASSERT(width > 0);
    DRAKE_ASSERT(height > 0);
  }

  /// Constructs a zero-sized image.
  Image() = default;

  /// Returns the size of width for the image
  int width() const { return width_; }

  /// Returns the size of height for the image
  int height() const { return height_; }

  /// Returns the number of channel for the image
  int num_channels() const { return channel; }

  /// Returns the result of the number of pixels in a image by the number of
  /// channels in a pixel
  int size() const { return width_ * height_ * channel; }

  /// Changes the sizes of the width and height for the image.  The values for
  /// them should be greater than zero.  (To resize to zero, assign a default-
  /// constructed Image into this; do not use this method.)  All the values in
  /// the pixels become zero after resize.
  void resize(int width, int height) {
    DRAKE_ASSERT(width > 0);
    DRAKE_ASSERT(height > 0);

    data_.resize(width * height * channel);
    std::fill(data_.begin(), data_.end(), 0);
    width_ = width;
    height_ = height;
  }

  /// Access to the pixel located at (x, y) in image coordinate system where x
  /// is the variable for horizontal direction and y is one for vertical
  /// direction.  To access to the each channel value in the pixel (x, y),
  /// you can do:
  ///
  /// ImageRgbaU8 image(640, 480, 255);
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

  PixelFormat pixel_format() const {
    return pixelformat;
  }

 private:
  reinit_after_move<int> width_;
  reinit_after_move<int> height_;
  std::vector<T> data_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
