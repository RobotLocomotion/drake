#include "drake/geometry/render/dev/render_gltf_client/test/internal_test_utility.h"

#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

namespace {
// These constexpr values are used to ensure the recreated images are
// dimensionally consistent with the test png or tiff files.
constexpr int kTestImageWidth = 3;
constexpr int kTestImageHeight = 2;
}  // namespace

ImageRgba8U CreateTestColorImage(int num_channel) {
  ImageRgba8U test_color_image{kTestImageWidth, kTestImageHeight};
  for (int j = 0; j < kTestImageHeight; ++j) {
    for (int i = 0; i < kTestImageWidth; ++i) {
      // The RGB pixel values will be 0, 51, 102, 153, 204, and 255 in the
      // row-major order.
      const auto pixel_value =
          static_cast<ImageRgba8U::T>(255 / 5 * (j * kTestImageWidth + i));
      test_color_image.at(i, j)[0] = pixel_value;
      test_color_image.at(i, j)[1] = pixel_value;
      test_color_image.at(i, j)[2] = pixel_value;
      // Use either the same value as RGB or 255 based on `num_channel`.
      test_color_image.at(i, j)[3] = (num_channel == 4 ? pixel_value : 255u);
    }
  }
  return test_color_image;
}

ImageDepth32F CreateTestDepthImage() {
  ImageDepth32F test_depth_image{kTestImageWidth, kTestImageHeight};
  for (int j = 0; j < kTestImageHeight; ++j) {
    for (int i = 0; i < kTestImageWidth; ++i) {
      test_depth_image.at(i, j)[0] =
          static_cast<ImageDepth32F::T>(j * kTestImageWidth + i);
    }
  }
  return test_depth_image;
}

ImageLabel16I CreateTestLabelImage() {
  ImageLabel16I test_label_image{kTestImageWidth, kTestImageHeight};
  for (int j = 0; j < kTestImageHeight; ++j) {
    for (int i = 0; i < kTestImageWidth; ++i) {
      test_label_image.at(i, j)[0] =
          static_cast<ImageLabel16I::T>(j * kTestImageWidth + i);
    }
  }
  return test_label_image;
}

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
