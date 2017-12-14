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

using Eigen::Isometry3d;

const int kWidth = 640;
const int kHeight = 480;
const double kZNear = 0.5;
const double kZFar = 5.;
const double kFovY = M_PI_4;
const bool kShowWindow = false;

// The following tolerance is used due to a precision difference between Ubuntu
// Linux and Mac OSX.
const double kColorPixelTolerance = 1.001;
const double kDepthTolerance = 1.1e-4;

// Holds `(x, y)` indices of the screen coordinate system where the ranges of
// `x` and `y` are [0, kWidth) and [0, kHeight) respectively.
struct ScreenCoord {
  int x{};
  int y{};
};

void CompareColor(const uint8_t* pixel,
                  const sensors::ColorI& color,
                  int alpha) {
  ASSERT_NEAR(pixel[0], color.r, kColorPixelTolerance);
  ASSERT_NEAR(pixel[1], color.g, kColorPixelTolerance);
  ASSERT_NEAR(pixel[2], color.b, kColorPixelTolerance);
  ASSERT_EQ(pixel[3], alpha);
}

// This suite tests RgbdRenderer.
template <class RGBD_RENDERER>
class RgbdRendererTest : public ::testing::Test {
 public:
  RgbdRendererTest() :
      color_(kWidth, kHeight), depth_(kWidth, kHeight), label_(kWidth, kHeight),
      // Looking strait down from 3m above the ground.
      X_WR_(Eigen::Translation3d(0, 0, kDefaultDistance) *
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
        CompareColor(color_.at(x, y), pixel, alpha);
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
    const auto& kTerrain = renderer_->get_flat_terrain_color();
    for (const auto& screen_coord : kOutliers) {
      const int x = screen_coord.x;
      const int y = screen_coord.y;
      // Color
      CompareColor(color_.at(x, y), kTerrain, 255u);
      // Depth
      ASSERT_NEAR(depth_.at(x, y)[0], kDefaultDistance, kDepthTolerance);
      // Label
      ASSERT_EQ(label_.at(x, y)[0], Label::kFlatTerrain);
    }
  }

  void SetUp() override {}

  // All tests on this class must invoke this first.
  void SetUp(const Eigen::Isometry3d& X_WR, bool add_terrain = false) {
    renderer_ = std::make_unique<RGBD_RENDERER>(
        X_WR, kWidth, kHeight, kZNear, kZFar, kFovY, kShowWindow);

    if (add_terrain)
      renderer_->AddFlatTerrain();
  }

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
  Isometry3d X_WR_;

  std::unique_ptr<RgbdRenderer> renderer_;
};

}  // namespace test
}  // namespace sensors
}  // namespace systems
}  // namespace drake
