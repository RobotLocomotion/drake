#include "drake/geometry/render_gltf_client/test/internal_sample_image_data.h"

#include <array>
#include <vector>

#include "drake/geometry/render/render_label.h"
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
  using T = ImageRgba8U::T;
  // clang-format off
  std::vector<std::array<T, 4>> image_data {
    {0, 1, 2, 3},     {10, 11, 12, 13}, {20, 21, 22, 23},
    {30, 31, 32, 33}, {40, 41, 42, 43}, {50, 51, 52, 53}
  };
  // clang-format on
  int p = 0;
  for (int y = 0; y < 2; ++y) {
    for (int x = 0; x < 3; ++x) {
      const auto& rgba = image_data[p];
      for (int c = 0; c < 3; ++c) test_color_image.at(x, y)[c] = rgba[c];
      test_color_image.at(x, y)[3] = pad_alpha ? T(255) : rgba[3];
      ++p;
    }
  }
  return test_color_image;
}

ImageDepth32F CreateTestDepthImage() {
  ImageDepth32F test_depth_image{kTestImageWidth, kTestImageHeight};
  using T = ImageDepth32F::T;
  std::vector<T> image_data{0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
  int p = 0;
  for (int y = 0; y < 2; ++y) {
    for (int x = 0; x < 3; ++x) {
      test_depth_image.at(x, y)[0] = image_data[p];
      ++p;
    }
  }
  return test_depth_image;
}

ImageLabel16I CreateTestLabelImage() {
  ImageLabel16I test_label_image{kTestImageWidth, kTestImageHeight};
  using T = ImageLabel16I::T;
  std::vector<T> image_data{0, 1, 2, 3, 4, render::RenderLabel::kDontCare};
  int p = 0;
  for (int y = 0; y < 2; ++y) {
    for (int x = 0; x < 3; ++x) {
      test_label_image.at(x, y)[0] = image_data[p];
      ++p;
    }
  }
  return test_label_image;
}

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
