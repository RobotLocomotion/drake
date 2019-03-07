#pragma once

#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {
namespace test {

// Holds `(x, y)` indices of the screen coordinate system where the ranges of
// `x` and `y` are [0, kWidth) and [0, kHeight) respectively.
struct ScreenCoord {
  int x{};
  int y{};
};

void CompareColor(const uint8_t* pixel,
                  const sensors::ColorI& color,
                  int alpha, double tolerance) {
  // Use ASSERT here instead of EXPECT_NEAR to stop all subsequent testing,
  // because this function could be called inside for loops to search over all
  // the pixels in an image.
  ASSERT_NEAR(pixel[0], color.r, tolerance);
  ASSERT_NEAR(pixel[1], color.g, tolerance);
  ASSERT_NEAR(pixel[2], color.b, tolerance);
  ASSERT_EQ(pixel[3], alpha);
}

// This suite tests RgbdRenderer.
template <class Renderer>
class RgbdRendererTest : public ::testing::Test {
 public:
  RgbdRendererTest() :
      color_(kWidth, kHeight), depth_(kWidth, kHeight), label_(kWidth, kHeight),
      // Looking strait down from 3m above the ground.
      X_WC_(Eigen::Translation3d(0, 0, kDefaultDistance) *
            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ())) {}

 protected:
  void Render() {
    renderer_->RenderColorImage(&color_);
    renderer_->RenderDepthImage(&depth_);
    renderer_->RenderLabelImage(&label_);
  }

  void VerifyUniformColor(const sensors::ColorI& pixel, int alpha) {
    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x) {
        CompareColor(color_.at(x, y), pixel, alpha, kColorPixelTolerance);
      }
    }
  }

  void VerifyUniformLabel(int16_t label) {
    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x) {
        ASSERT_EQ(label_.at(x, y)[0], label);
      }
    }
  }

  void VerifyUniformDepth(float depth) {
    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x) {
        ASSERT_NEAR(depth_.at(x, y)[0], depth, kDepthTolerance);
      }
    }
  }

  // Verifies the chosen pixels, i.e. `kOutliers`, belong to the ground.
  void VerifyOutliers() {
    const auto& kTerrain = renderer_->color_palette().get_terrain_color();
    for (const auto& screen_coord : kOutliers) {
      const int x = screen_coord.x;
      const int y = screen_coord.y;
      // Color
      CompareColor(color_.at(x, y), kTerrain, 255u, kColorPixelTolerance);
      // Depth
      ASSERT_NEAR(depth_.at(x, y)[0], kDefaultDistance, kDepthTolerance);
      // Label
      ASSERT_EQ(label_.at(x, y)[0], Label::kFlatTerrain);
    }
  }

  // All tests on this class must invoke this first.
  void Init(const Eigen::Isometry3d& X_WC, bool add_terrain = false) {
    renderer_ = std::make_unique<Renderer>(
        RenderingConfig{kWidth, kHeight, kFovY, kZNear, kZFar, kShowWindow},
        X_WC);

    if (add_terrain) renderer_->AddFlatTerrain();
  }

  const int kWidth = 640;
  const int kHeight = 480;
  const double kZNear = 0.5;
  const double kZFar = 5.;
  const double kFovY = M_PI_4;
  const bool kShowWindow = RenderingConfig::kDefaultShowWindow;

  // The following tolerance is used due to a precision difference between
  // Ubuntu Linux and Mac OSX.
  const double kColorPixelTolerance = 1.001;
  const double kDepthTolerance = 1.1e-4;

  const sensors::ColorI kDefaultVisualColor = {179u, 179u, 179u};
  const float kDefaultDistance{3.f};
  // `kInlier` is chosen to point to a pixel representing an object in the
  // test scene.
  const ScreenCoord kInlier = {320, 240};
  // The outliers are chosen to point to pixels representing the ground,
  // not objects in the test scene.
  const std::array<ScreenCoord, 4> kOutliers{{
      {10, 10}, {10, 470}, {630, 10}, {630, 470}}};

  ImageRgba8U color_;
  ImageDepth32F depth_;
  ImageLabel16I label_;
  Eigen::Isometry3d X_WC_;

  std::unique_ptr<RgbdRenderer> renderer_;
};

}  // namespace test
}  // namespace sensors
}  // namespace systems
}  // namespace drake
