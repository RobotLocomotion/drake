#include "drake/geometry/render/dev/render_gltf_client/test/internal_sample_image_data.h"

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

ImageRgba8U CreateTestColorImage(bool pad_alpha) {
  ImageRgba8U test_color_image{kTestImageWidth, kTestImageHeight};
  test_color_image.at(0, 0)[0] = static_cast<ImageRgba8U::T>(0);
  test_color_image.at(0, 0)[1] = static_cast<ImageRgba8U::T>(1);
  test_color_image.at(0, 0)[2] = static_cast<ImageRgba8U::T>(2);
  test_color_image.at(0, 0)[3] =
      static_cast<ImageRgba8U::T>(pad_alpha ? 255 : 3);
  test_color_image.at(1, 0)[0] = static_cast<ImageRgba8U::T>(10);
  test_color_image.at(1, 0)[1] = static_cast<ImageRgba8U::T>(11);
  test_color_image.at(1, 0)[2] = static_cast<ImageRgba8U::T>(12);
  test_color_image.at(1, 0)[3] =
      static_cast<ImageRgba8U::T>(pad_alpha ? 255 : 13);
  test_color_image.at(2, 0)[0] = static_cast<ImageRgba8U::T>(20);
  test_color_image.at(2, 0)[1] = static_cast<ImageRgba8U::T>(21);
  test_color_image.at(2, 0)[2] = static_cast<ImageRgba8U::T>(22);
  test_color_image.at(2, 0)[3] =
      static_cast<ImageRgba8U::T>(pad_alpha ? 255 : 23);
  test_color_image.at(0, 1)[0] = static_cast<ImageRgba8U::T>(30);
  test_color_image.at(0, 1)[1] = static_cast<ImageRgba8U::T>(31);
  test_color_image.at(0, 1)[2] = static_cast<ImageRgba8U::T>(32);
  test_color_image.at(0, 1)[3] =
      static_cast<ImageRgba8U::T>(pad_alpha ? 255 : 33);
  test_color_image.at(1, 1)[0] = static_cast<ImageRgba8U::T>(40);
  test_color_image.at(1, 1)[1] = static_cast<ImageRgba8U::T>(41);
  test_color_image.at(1, 1)[2] = static_cast<ImageRgba8U::T>(42);
  test_color_image.at(1, 1)[3] =
      static_cast<ImageRgba8U::T>(pad_alpha ? 255 : 43);
  test_color_image.at(2, 1)[0] = static_cast<ImageRgba8U::T>(50);
  test_color_image.at(2, 1)[1] = static_cast<ImageRgba8U::T>(51);
  test_color_image.at(2, 1)[2] = static_cast<ImageRgba8U::T>(52);
  test_color_image.at(2, 1)[3] =
      static_cast<ImageRgba8U::T>(pad_alpha ? 255 : 53);
  return test_color_image;
}

ImageDepth32F CreateTestDepthImage() {
  ImageDepth32F test_depth_image{kTestImageWidth, kTestImageHeight};
  test_depth_image.at(0, 0)[0] = static_cast<ImageDepth32F::T>(0);
  test_depth_image.at(1, 0)[0] = static_cast<ImageDepth32F::T>(1);
  test_depth_image.at(2, 0)[0] = static_cast<ImageDepth32F::T>(2);
  test_depth_image.at(0, 1)[0] = static_cast<ImageDepth32F::T>(3);
  test_depth_image.at(1, 1)[0] = static_cast<ImageDepth32F::T>(4);
  test_depth_image.at(2, 1)[0] = static_cast<ImageDepth32F::T>(5);
  return test_depth_image;
}

ImageLabel16I CreateTestLabelImage() {
  ImageLabel16I test_label_image{kTestImageWidth, kTestImageHeight};
  test_label_image.at(0, 0)[0] = static_cast<ImageLabel16I::T>(0);
  test_label_image.at(1, 0)[0] = static_cast<ImageLabel16I::T>(1);
  test_label_image.at(2, 0)[0] = static_cast<ImageLabel16I::T>(2);
  test_label_image.at(0, 1)[0] = static_cast<ImageLabel16I::T>(3);
  test_label_image.at(1, 1)[0] = static_cast<ImageLabel16I::T>(4);
  test_label_image.at(2, 1)[0] = static_cast<ImageLabel16I::T>(5);
  return test_label_image;
}

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
