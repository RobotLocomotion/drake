#include "drake/systems/sensors/pixel_types.h"

namespace drake {
namespace systems {
namespace sensors {

constexpr int ImageTraits<PixelType::kRgb8U>::kNumChannels;
constexpr PixelFormat ImageTraits<PixelType::kRgb8U>::kPixelFormat;

constexpr int ImageTraits<PixelType::kBgr8U>::kNumChannels;
constexpr PixelFormat ImageTraits<PixelType::kBgr8U>::kPixelFormat;

constexpr int ImageTraits<PixelType::kRgba8U>::kNumChannels;
constexpr PixelFormat ImageTraits<PixelType::kRgba8U>::kPixelFormat;

constexpr int ImageTraits<PixelType::kBgra8U>::kNumChannels;
constexpr PixelFormat ImageTraits<PixelType::kBgra8U>::kPixelFormat;

constexpr int ImageTraits<PixelType::kDepth32F>::kNumChannels;
constexpr PixelFormat ImageTraits<PixelType::kDepth32F>::kPixelFormat;
constexpr ImageTraits<PixelType::kDepth32F>::ChannelType ImageTraits<PixelType::kDepth32F>::kTooClose;  // NOLINT
constexpr ImageTraits<PixelType::kDepth32F>::ChannelType ImageTraits<PixelType::kDepth32F>::kTooFar;  // NOLINT

constexpr int ImageTraits<PixelType::kDepth16U>::kNumChannels;
constexpr PixelFormat ImageTraits<PixelType::kDepth16U>::kPixelFormat;
constexpr ImageTraits<PixelType::kDepth16U>::ChannelType ImageTraits<PixelType::kDepth16U>::kTooClose;  // NOLINT
constexpr ImageTraits<PixelType::kDepth16U>::ChannelType ImageTraits<PixelType::kDepth16U>::kTooFar;  // NOLINT

constexpr int ImageTraits<PixelType::kLabel16I>::kNumChannels;
constexpr PixelFormat ImageTraits<PixelType::kLabel16I>::kPixelFormat;

constexpr int ImageTraits<PixelType::kGrey8U>::kNumChannels;
constexpr PixelFormat ImageTraits<PixelType::kGrey8U>::kPixelFormat;

constexpr int ImageTraits<PixelType::kExpr>::kNumChannels;
constexpr PixelFormat ImageTraits<PixelType::kExpr>::kPixelFormat;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
