#pragma once

#include <cstdint>

#include "robotlocomotion/image_t.hpp"

#include "drake/systems/sensors/pixel_types.h"

namespace drake {
namespace systems {
namespace sensors {

template <PixelFormat>
struct LcmPixelTraits;

template <>
struct LcmPixelTraits<PixelFormat::kRgb> {
  static constexpr uint8_t kPixelFormat =
      robotlocomotion::image_t::PIXEL_FORMAT_RGB;
};

template <>
struct LcmPixelTraits<PixelFormat::kBgr> {
  static constexpr uint8_t kPixelFormat =
      robotlocomotion::image_t::PIXEL_FORMAT_BGR;
};

template <>
struct LcmPixelTraits<PixelFormat::kRgba> {
  static constexpr uint8_t kPixelFormat =
      robotlocomotion::image_t::PIXEL_FORMAT_RGBA;
};

template <>
struct LcmPixelTraits<PixelFormat::kBgra> {
  static constexpr uint8_t kPixelFormat =
      robotlocomotion::image_t::PIXEL_FORMAT_BGRA;
};

template <>
struct LcmPixelTraits<PixelFormat::kGrey> {
  static constexpr uint8_t kPixelFormat =
      robotlocomotion::image_t::PIXEL_FORMAT_GRAY;
};

template <>
struct LcmPixelTraits<PixelFormat::kDepth> {
  static constexpr uint8_t kPixelFormat =
      robotlocomotion::image_t::PIXEL_FORMAT_DEPTH;
};

template <>
struct LcmPixelTraits<PixelFormat::kLabel> {
  static constexpr uint8_t kPixelFormat =
      robotlocomotion::image_t::PIXEL_FORMAT_LABEL;
};

template <PixelType>
struct LcmImageTraits;

template <>
struct LcmImageTraits<PixelType::kRgb8U> {
  static constexpr uint8_t kChannelType =
      robotlocomotion::image_t::CHANNEL_TYPE_UINT8;
};

template <>
struct LcmImageTraits<PixelType::kBgr8U> {
  static constexpr uint8_t kChannelType =
      robotlocomotion::image_t::CHANNEL_TYPE_UINT8;
};

template <>
struct LcmImageTraits<PixelType::kRgba8U> {
  static constexpr uint8_t kChannelType =
      robotlocomotion::image_t::CHANNEL_TYPE_UINT8;
};

template <>
struct LcmImageTraits<PixelType::kBgra8U> {
  static constexpr uint8_t kChannelType =
      robotlocomotion::image_t::CHANNEL_TYPE_UINT8;
};

template <>
struct LcmImageTraits<PixelType::kGrey8U> {
  static constexpr uint8_t kChannelType =
      robotlocomotion::image_t::CHANNEL_TYPE_UINT8;
};

template <>
struct LcmImageTraits<PixelType::kDepth16U> {
  static constexpr uint8_t kChannelType =
      robotlocomotion::image_t::CHANNEL_TYPE_UINT16;
};

template <>
struct LcmImageTraits<PixelType::kDepth32F> {
  static constexpr uint8_t kChannelType =
      robotlocomotion::image_t::CHANNEL_TYPE_FLOAT32;
};

template <>
struct LcmImageTraits<PixelType::kLabel16I> {
  static constexpr uint8_t kChannelType =
      robotlocomotion::image_t::CHANNEL_TYPE_INT16;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
