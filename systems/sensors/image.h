#pragma once

#include <limits>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/reset_after_move.h"
#include "drake/systems/sensors/pixel_types.h"

namespace drake {
namespace systems {
namespace sensors {

// Forward declaration of Image class.
template <PixelType kPixelType>
class Image;

/// The type for RGB image where the each channel has the type of uint8_t.
using ImageRgb8U = Image<PixelType::kRgb8U>;

/// The type for BGR image where the each channel has the type of uint8_t.
using ImageBgr8U = Image<PixelType::kBgr8U>;

/// The type for RGBA image where the each channel has the type of uint8_t.
using ImageRgba8U = Image<PixelType::kRgba8U>;

/// The type for BGRA image where the each channel has the type of uint8_t.
using ImageBgra8U = Image<PixelType::kBgra8U>;

/// The type for depth image where the channel has the type of float.
using ImageDepth32F = Image<PixelType::kDepth32F>;

/// The type for depth image where the channel has the type of uint16_t.
using ImageDepth16U = Image<PixelType::kDepth16U>;

/// The type for label image where the channel has the type of int16_t.
using ImageLabel16I = Image<PixelType::kLabel16I>;

/// The type for greyscale image where the channel has the type of uint8_t.
using ImageGrey8U = Image<PixelType::kGrey8U>;

/// The type for symbolic image where the channel has the type of
/// symbolic::Expression.
using ImageExpr = Image<PixelType::kExpr>;

/// Simple data format for Image. For the complex calculation with the image,
/// consider converting this to other libaries' Matrix data format, i.e.,
/// MatrixX in Eigen, Mat in OpenCV, and so on.
///
/// The origin of image coordinate system is on the left-upper corner.
///
/// @tparam kPixelType The pixel type enum that denotes the pixel format and the
/// data type of a channel.
template <PixelType kPixelType>
class Image {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Image)

  /// This is used by generic helpers such as drake::Value to deduce a non-type
  /// template argument.
  using NonTypeTemplateParameter =
      std::integral_constant<PixelType, kPixelType>;

  /// An alias for ImageTraits that contains the data type for a channel,
  /// the number of channels and the pixel format in it.
  using Traits = ImageTraits<kPixelType>;

  /// The data type for a channel.
  using T = typename Traits::ChannelType;

  /// The number of channels in a pixel.
  static constexpr int kNumChannels = Traits::kNumChannels;

  /// The size of a pixel in bytes.
  static constexpr int kPixelSize = kNumChannels * sizeof(T);

  /// The format for pixels.
  static constexpr PixelFormat kPixelFormat = Traits::kPixelFormat;

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
        data_(width * height * kNumChannels, initial_value) {
    DRAKE_ASSERT(width > 0);
    DRAKE_ASSERT(height > 0);
  }

  /// Constructs a zero-sized image.
  Image() = default;

  /// Returns the size of width for the image
  int width() const { return width_; }

  /// Returns the size of height for the image
  int height() const { return height_; }

  /// Returns the result of the number of pixels in a image by the number of
  /// channels in a pixel
  int size() const { return width_ * height_ * kNumChannels; }

  /// Changes the sizes of the width and height for the image.  The values for
  /// them should be greater than zero.  (To resize to zero, assign a default-
  /// constructed Image into this; do not use this method.)  All the values in
  /// the pixels become zero after resize.
  void resize(int width, int height) {
    DRAKE_ASSERT(width > 0);
    DRAKE_ASSERT(height > 0);

    data_.resize(width * height * kNumChannels);
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
    return data_.data() + (x + y * width_) * kNumChannels;
  }

  /// Const version of at() method.  See the document for the non-const version
  /// for the detail.
  const T* at(int x, int y) const {
    DRAKE_ASSERT(x >= 0 && x < width_);
    DRAKE_ASSERT(y >= 0 && y < height_);
    return data_.data() + (x + y * width_) * kNumChannels;
  }

 private:
  reset_after_move<int> width_;
  reset_after_move<int> height_;
  std::vector<T> data_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
