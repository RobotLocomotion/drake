#include "drake/systems/sensors/rgbd_renderer.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {

using Eigen::Isometry3d;

const int kWidth = 640;
const int kHeight = 480;
const double kZNear = 0.5;
const double kZFar = 5.;
const double kFovY = M_PI_4;
const bool kShowWindow = true;

// The following tolerance is used due to a precision difference between Ubuntu
// Linux and Mac OSX.
const double kColorPixelTolerance = 1.001;
const double kDepthTolerance = 1e-4;

struct UV {
  UV(int x, int y) : u(x), v(y) {}
  int u;
  int v;
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

  void VerifyColor(const sensors::ColorI& pixel, int alpha) {
    for (int v = 0; v < kHeight; ++v) {
      for (int u = 0; u < kWidth; ++u) {
        CompareColor(color_.at(u, v), pixel, alpha);
      }
    }
  }

  void VerifyLabel(int16_t label) {
    for (int v = 0; v < kHeight; ++v) {
      for (int u = 0; u < kWidth; ++u) {
        ASSERT_EQ(label_.at(u, v)[0], label);
      }
    }
  }

  void VerifyDepth(float depth) {
    for (int v = 0; v < kHeight; ++v) {
      for (int u = 0; u < kWidth; ++u) {
        ASSERT_NEAR(depth_.at(u, v)[0], depth, kDepthTolerance);
      }
    }
  }

  void VerifyOutliers() {
    const auto& kTerrain = renderer_->get_flat_terrain_color();
    for (const auto& pixcord : kOutliers) {
      const int u = pixcord.u;
      const int v = pixcord.v;
      // Color
      CompareColor(color_.at(u, v), kTerrain, 255u);
      // Depth
      ASSERT_NEAR(depth_.at(u, v)[0], kDefaultDistance, kDepthTolerance);
      // Label
      ASSERT_EQ(label_.at(u, v)[0], RgbdRenderer::Label::kFlatTerrain);
    }
  }

  void SetUp() override {}

  void SetUp(const Eigen::Isometry3d& X_WR, bool add_terrain = false) {
    renderer_ = std::make_unique<RgbdRenderer>(
        X_WR, kWidth, kHeight, kZNear, kZFar, kFovY, kShowWindow);

    if (add_terrain)
      renderer_->AddFlatTerrain();
  }

  const sensors::ColorI kDefaultVisualColor = {179u, 179u, 179u};
  const float kDefaultDistance{3.f};
  const UV kInlier = {320, 240};
  const std::array<UV, 4> kOutliers{{
      UV(10, 10), UV(10, 470), UV(630, 10), UV(630, 470)}};

  ImageRgba8U color_;
  ImageDepth32F depth_;
  ImageLabel16I label_;
  Isometry3d X_WR_;

  std::unique_ptr<RgbdRenderer> renderer_;
};


TEST_F(RgbdRendererTest, InstantiationTest) {
  SetUp(Isometry3d::Identity());

  EXPECT_EQ(renderer_->width(), kWidth);
  EXPECT_EQ(renderer_->height(), kHeight);
  EXPECT_EQ(renderer_->fov_y(), kFovY);
}


TEST_F(RgbdRendererTest, NoBodyTest) {
  SetUp(Isometry3d::Identity());
  Render();

  VerifyColor(renderer_->get_sky_color(), 0u);
  VerifyLabel(RgbdRenderer::Label::kNoBody);
  // Verifies depth.
  for (int v = 0; v < kHeight; ++v) {
    for (int u = 0; u < kWidth; ++u) {
      EXPECT_TRUE(std::isnan(depth_.at(u, v)[0]));
    }
  }
}


TEST_F(RgbdRendererTest, TerrainTest) {
  SetUp(X_WR_, true);

  const auto& kTerrain = renderer_->get_flat_terrain_color();
  // At two different distances.
  for (auto distance : std::array<float, 2>({{2.f, 5.f}})) {
    X_WR_.translation().z() = distance;
    renderer_->UpdateViewpoint(X_WR_);
    Render();
    VerifyColor(kTerrain, 255u);
    VerifyLabel(RgbdRenderer::Label::kFlatTerrain);
    VerifyDepth(distance);
  }

  // Closer than kZNear.
  X_WR_.translation().z() = kZNear - 1e-5;
  renderer_->UpdateViewpoint(X_WR_);
  Render();
  VerifyColor(kTerrain, 255u);
  VerifyLabel(RgbdRenderer::Label::kFlatTerrain);
  VerifyDepth(RgbdRenderer::InvalidDepth::kTooClose);

  // Further than kZFar.
  X_WR_.translation().z() = kZFar + 1e-3;
  renderer_->UpdateViewpoint(X_WR_);
  Render();
  VerifyColor(kTerrain, 255u);
  VerifyLabel(RgbdRenderer::Label::kFlatTerrain);
  // Verifies depth.
  for (int v = 0; v < kHeight; ++v) {
    for (int u = 0; u < kWidth; ++u) {
      ASSERT_TRUE(std::isinf(depth_.at(u, v)[0]));
    }
  }
}

TEST_F(RgbdRendererTest, HorizonTest) {
  // Looking at horizon with the ray direction parallel to the ground.
  Isometry3d X_WR =
      Eigen::Translation3d(0, 0, 0) *
      Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
  SetUp(X_WR, true);

  // Returns v index of horizon location in image coordinate system.
  auto CalcHorizon = [](double z) {
    const double kTerrainSize = 50.;
    const double kFocalLength = kHeight * 0.5 / std::tan(0.5 * kFovY);
    return 0.5 * kHeight + z / kTerrainSize * kFocalLength;
  };

  // Verifies v index of horizon at three different camera heights.
  const std::array<double, 3> Zs{{2., 1., 0.5}};
  for (const auto& z : Zs) {
    X_WR.translation().z() = z;
    renderer_->UpdateViewpoint(X_WR);
    Render();

    int actual_horizon{0};
    std::array<uint8_t, 4> a_pixel{{0u, 0u, 0u, 0u}};
    for (int v = 0; v < kHeight; ++v) {
      if ((a_pixel[0] != color_.at(0, v)[0]) ||
          (a_pixel[1] != color_.at(0, v)[1]) ||
          (a_pixel[2] != color_.at(0, v)[2]) ||
          (a_pixel[3] != color_.at(0, v)[3])) {
        for (int ch = 0; ch < color_.kNumChannels; ++ch) {
          a_pixel[ch] = color_.at(0, v)[ch];
        }
        actual_horizon = v;
      }
    }

    const double expected_horizon = CalcHorizon(z);
    ASSERT_NEAR(expected_horizon, actual_horizon, 1.001);
  }
}


TEST_F(RgbdRendererTest, BoxTest) {
  SetUp(X_WR_, true);

  // Sets up a box.
  Isometry3d X_WV = Isometry3d::Identity();
  X_WV.translation().z() = 0.5;
  DrakeShapes::VisualElement visual(X_WV);
  Eigen::Vector3d xyz(1, 1, 1);
  visual.setGeometry(DrakeShapes::Box(xyz));
  const int kBodyID = 0;
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, 0);
  Render();

  VerifyOutliers();

  // Verifies inside the box.
  const int u = kInlier.u;
  const int v = kInlier.v;
  // Color
  CompareColor(color_.at(u, v), kDefaultVisualColor, 255u);
  // Depth
  ASSERT_NEAR(depth_.at(u, v)[0], 2.f, kDepthTolerance);
  // Label
  ASSERT_EQ(label_.at(u, v)[0], kBodyID);
}


TEST_F(RgbdRendererTest, SphereTest) {
  SetUp(X_WR_, true);

  // Sets up a sphere.
  Isometry3d X_WV = Isometry3d::Identity();
  X_WV.translation().z() = 0.5;
  DrakeShapes::VisualElement visual(X_WV);
  visual.setGeometry(DrakeShapes::Sphere(0.5));
  const int kBodyID = 0;
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, 0);
  Render();

  VerifyOutliers();

  // Verifies inside the sphere.
  const int u = kInlier.u;
  const int v = kInlier.v;
  // Color
  CompareColor(color_.at(u, v), kDefaultVisualColor, 255u);
  // Depth
  ASSERT_NEAR(depth_.at(u, v)[0], 2.f, kDepthTolerance);
  // Label
  ASSERT_EQ(label_.at(u, v)[0], kBodyID);
}


TEST_F(RgbdRendererTest, CylinderTest) {
  SetUp(X_WR_, true);

  // Sets up a sphere.
  Isometry3d X_WV = Isometry3d::Identity();
  X_WV.translation().z() = 0.6;
  DrakeShapes::VisualElement visual(X_WV);
  visual.setGeometry(DrakeShapes::Cylinder(0.2, 1.2));  // Radius and length.
  const int kBodyID = 1;
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, 0);
  Render();

  VerifyOutliers();

  // Verifies inside the cylinder.
  const int u = kInlier.u;
  const int v = kInlier.v;
  // Color
  CompareColor(color_.at(u, v), kDefaultVisualColor, 255u);
  // Depth
  ASSERT_NEAR(depth_.at(u, v)[0], 1.8f, kDepthTolerance);
  // Label
  ASSERT_EQ(label_.at(u, v)[0], kBodyID);
}


TEST_F(RgbdRendererTest, MeshTest) {
  SetUp(X_WR_, true);

  Isometry3d X_WV = Isometry3d::Identity();
  DrakeShapes::VisualElement visual(X_WV);
  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  visual.setGeometry(DrakeShapes::Mesh("", filename));
  const int kBodyID = 0;
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, 0);
  Render();

  VerifyOutliers();

  // Verifies inside the cylinder.
  const int u = kInlier.u;
  const int v = kInlier.v;
  // Color
  CompareColor(color_.at(u, v), ColorI({4u, 241u, 33u}), 255u);
  // Depth
  ASSERT_NEAR(depth_.at(u, v)[0], 2.f, kDepthTolerance);
  // Label
  ASSERT_EQ(label_.at(u, v)[0], kBodyID);
}

// TODO(kunimatsu-tri) Move DepthImageToPointCloudConversionTest here from
// rgbd_camera_test.

}  // anonymous namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
