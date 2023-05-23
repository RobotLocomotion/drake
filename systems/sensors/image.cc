#include "drake/systems/sensors/image.h"

#include <algorithm>

namespace drake {
namespace systems {
namespace sensors {

namespace {
/* Converts an input depth image to an output depth image using the given
per-pixel conversion function. */
template <typename InputImage, typename OutputImage, typename Func>
void ConvertDepthPixelwise(const InputImage& input, OutputImage* output,
                           Func func) {
  const int width = input.width();
  const int height = input.height();
  if (!(output->width() == width && output->height() == height)) {
    output->resize(width, height);
  }
  const int size = input.size();
  if (size == 0) {
    return;
  }
  const typename InputImage::T* const in = input.at(0, 0);
  typename OutputImage::T* const out = output->at(0, 0);
  for (int i = 0; i < size; ++i) {
    out[i] = func(in[i]);
  }
}
}  // namespace

// N.B. The `output` images below are phrased as an output argument rather than
// a return value to allow the caller to reuse the same storage from one Convert
// call to the next.

void ConvertDepth32FTo16U(const ImageDepth32F& input, ImageDepth16U* output) {
  DRAKE_THROW_UNLESS(output != nullptr);
  ConvertDepthPixelwise(
      input, output, [](const ImageDepth32F::T pixel) -> ImageDepth16U::T {
        // Start by scaling from meters to millimeters, keeping all math as
        // 32-bit floats to mitigate roundoff errors.
        float result = pixel * 1000.0f;
        // Clamp too-far values (including +Inf and NaN) to the 16-bit constant.
        result = std::min<float>(ImageDepth16U::Traits::kTooFar, result);
        // Clamp too-close values (including -Inf) to the 16-bit constant.
        result = std::max<float>(ImageDepth16U::Traits::kTooClose, result);
        // Truncate towards zero.
        return result;
      });
}

void ConvertDepth16UTo32F(const ImageDepth16U& input, ImageDepth32F* output) {
  DRAKE_THROW_UNLESS(output != nullptr);
  ConvertDepthPixelwise(
      input, output, [](const ImageDepth16U::T pixel) -> ImageDepth32F::T {
        using InPixel = ImageDepth16U::Traits;
        using OutPixel = ImageDepth32F::Traits;
        // No special case is necessary for kTooClose.
        static_assert(InPixel::kTooClose == 0 && OutPixel::kTooClose == 0);
        // Scale from millimeters to meters, with kTooFar as a special case,
        // using intermediate 64-bit doubles to mitigate roundoff errors.
        return (pixel == InPixel::kTooFar) ? OutPixel::kTooFar : 0.001 * pixel;
      });
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
