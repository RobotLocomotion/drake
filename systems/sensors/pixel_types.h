#pragma once

#include <cstdint>

namespace drake {
namespace systems {
namespace sensors {

/// The enum class to be used for describing pixel type in Image class.
/// The naming rule for the enum members is:
/// k + (pixel format) + (bit per a channel) + (data type for channels).
/// For the type for channels, one of the following capital letters is used.
///   - I: int
///   - U: unsigned int
///   - F: float
enum class PixelType {
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


/// The enum class to be used to express semantic meaning of pixels.
/// This also expresses the order of channels in a pixel if the pixel has
/// multiple channels.
enum class PixelFormat {
  /// The pixel format used for all the RGB images.
  kRgb = 0,
  /// The pixel format used for all the BGR images.
  kBgr,
  /// The pixel format used for all the RGBA images.
  kRgba,
  /// The pixel format used for all the BGRA images.
  kBgra,
  /// The pixel format used for all the greyscale images.
  kGrey,
  /// The pixel format used for all the depth images.
  kDepth,
  /// The pixel format used for all the labe images.
  kLabel,
};

/// Traits class for Image.
template <PixelType>
struct ImageTraits;

template <>
struct ImageTraits<PixelType::kRgb8U> {
  typedef uint8_t ChannelType;
  static constexpr int kNumChannels = 3;
  static constexpr PixelFormat kPixelFormat = PixelFormat::kRgb;
};

template <>
struct ImageTraits<PixelType::kBgr8U> {
  typedef uint8_t ChannelType;
  static constexpr int kNumChannels = 3;
  static constexpr PixelFormat kPixelFormat = PixelFormat::kBgr;
};

template <>
struct ImageTraits<PixelType::kRgba8U> {
  typedef uint8_t ChannelType;
  static constexpr int kNumChannels = 4;
  static constexpr PixelFormat kPixelFormat = PixelFormat::kRgba;
};

template <>
struct ImageTraits<PixelType::kBgra8U> {
  typedef uint8_t ChannelType;
  static constexpr int kNumChannels = 4;
  static constexpr PixelFormat kPixelFormat = PixelFormat::kBgra;
};

template <>
struct ImageTraits<PixelType::kDepth32F> {
  typedef float ChannelType;
  static constexpr int kNumChannels = 1;
  static constexpr PixelFormat kPixelFormat = PixelFormat::kDepth;
};

template <>
struct ImageTraits<PixelType::kDepth16U> {
  typedef uint16_t ChannelType;
  static constexpr int kNumChannels = 1;
  static constexpr PixelFormat kPixelFormat = PixelFormat::kDepth;
};

template <>
struct ImageTraits<PixelType::kLabel16I> {
  typedef int16_t ChannelType;
  static constexpr int kNumChannels = 1;
  static constexpr PixelFormat kPixelFormat = PixelFormat::kLabel;
};

template <>
struct ImageTraits<PixelType::kGrey8U> {
  typedef uint8_t ChannelType;
  static constexpr int kNumChannels = 1;
  static constexpr PixelFormat kPixelFormat = PixelFormat::kGrey;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
