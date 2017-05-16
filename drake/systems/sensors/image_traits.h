#pragma once

#include <cstdint>

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


/// Traits class for Image.
template <PixelFormat>
struct ImageTraits;

template <>
struct ImageTraits<PixelFormat::kRgb8U> {
  typedef uint8_t ChannelType;
  static constexpr int channel = 3;
};

template <>
struct ImageTraits<PixelFormat::kBgr8U> {
  typedef uint8_t ChannelType;
  static constexpr int channel = 3;
};

template <>
struct ImageTraits<PixelFormat::kRgba8U> {
  typedef uint8_t ChannelType;
  static constexpr int channel = 4;
};

template <>
struct ImageTraits<PixelFormat::kBgra8U> {
  typedef uint8_t ChannelType;
  static constexpr int channel = 4;
};

template <>
struct ImageTraits<PixelFormat::kDepth32F> {
  typedef float ChannelType;
  static constexpr int channel = 1;
};

template <>
struct ImageTraits<PixelFormat::kDepth16U> {
  typedef uint16_t ChannelType;
  static constexpr int channel = 1;
};

template <>
struct ImageTraits<PixelFormat::kLabel16I> {
  typedef int16_t ChannelType;
  static constexpr int channel = 1;
};

template <>
struct ImageTraits<PixelFormat::kGrey8U> {
  typedef uint8_t ChannelType;
  static constexpr int channel = 1;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
