#include "drake/systems/sensors/rgbd_renderer_vtk.h"

#include "drake/systems/sensors/test/rgbd_renderer_test_util.h"

namespace drake {
namespace systems {
namespace sensors {
namespace test {

using RgbdRendererVTKTest = RgbdRendererTest<RgbdRendererVTK>;
using Eigen::Isometry3d;

TEST_F(RgbdRendererVTKTest, InstantiationTest) {
  Init(Isometry3d::Identity());

  EXPECT_EQ(renderer_->config().width, kWidth);
  EXPECT_EQ(renderer_->config().height, kHeight);
  EXPECT_EQ(renderer_->config().fov_y, kFovY);
}

TEST_F(RgbdRendererVTKTest, NoBodyTest) {
  Init(Isometry3d::Identity());
  Render();

  VerifyUniformColor(renderer_->color_palette().get_sky_color(), 0u);
  VerifyUniformLabel(Label::kNoBody);
  // Verifies depth.
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      // Using ASSERT here instead of EXPECT to stop all subsequent testing,
      // because this function has been called inside for loops to search over
      // all the pixels in an image.
      ASSERT_TRUE(std::isinf(depth_.at(x, y)[0]));
    }
  }
}

TEST_F(RgbdRendererVTKTest, TerrainTest) {
  Init(X_WC_, true);

  const auto& kTerrain = renderer_->color_palette().get_terrain_color();
  // At two different distances.
  for (auto depth : std::array<float, 2>({{2.f, 4.9999f}})) {
    X_WC_.translation().z() = depth;
    renderer_->UpdateViewpoint(X_WC_);
    Render();
    VerifyUniformColor(kTerrain, 255u);
    VerifyUniformLabel(Label::kFlatTerrain);
    VerifyUniformDepth(depth);
  }

  // Closer than kZNear.
  X_WC_.translation().z() = kZNear - 1e-5;
  renderer_->UpdateViewpoint(X_WC_);
  Render();
  VerifyUniformColor(kTerrain, 255u);
  VerifyUniformLabel(Label::kFlatTerrain);
  VerifyUniformDepth(InvalidDepth::kTooClose);

  // Farther than kZFar.
  X_WC_.translation().z() = kZFar + 1e-3;
  renderer_->UpdateViewpoint(X_WC_);
  Render();
  VerifyUniformColor(kTerrain, 255u);
  VerifyUniformLabel(Label::kFlatTerrain);
  // Verifies depth.
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      ASSERT_EQ(InvalidDepth::kTooFar, depth_.at(x, y)[0]);
    }
  }
}

TEST_F(RgbdRendererVTKTest, HorizonTest) {
  // Camera at the origin, pointing in a direction parallel to the ground.
  Isometry3d X_WC =
      Eigen::Translation3d(0, 0, 0) *
      Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
  Init(X_WC, true);

  // Returns y in [0, kHeight / 2], index of horizon location in image
  // coordinate system under two assumptions: 1) the gound plane is not clipped
  // by `kClippingPlaneFar`, 2) camera is located above the ground.
  auto CalcHorizon = [](double z, double fov, double height) {
    const double kTerrainSize = 50.;
    const double kFocalLength = height * 0.5 / std::tan(0.5 * fov);
    return 0.5 * height + z / kTerrainSize * kFocalLength;
  };

  // Verifies v index of horizon at three different camera heights.
  const std::array<double, 3> Zs{{2., 1., 0.5}};
  for (const auto& z : Zs) {
    X_WC.translation().z() = z;
    renderer_->UpdateViewpoint(X_WC);
    Render();

    const auto& kTerrain = renderer_->color_palette().get_terrain_color();
    int actual_horizon{0};
    for (int y = 0; y < kHeight; ++y) {
      // Looking for the boundary between the sky and the ground.
      if ((static_cast<uint8_t>(kTerrain.r == color_.at(0, y)[0])) &&
          (static_cast<uint8_t>(kTerrain.g == color_.at(0, y)[1])) &&
          (static_cast<uint8_t>(kTerrain.b == color_.at(0, y)[2]))) {
        actual_horizon = y;
        break;
      }
    }

    const double expected_horizon = CalcHorizon(z, kFovY, kHeight);
    ASSERT_NEAR(expected_horizon, actual_horizon, 1.001);
  }
}

TEST_F(RgbdRendererVTKTest, BoxTest) {
  Init(X_WC_, true);

  // Sets up a box.
  Isometry3d X_WV = Isometry3d::Identity();
  X_WV.translation().z() = 0.5;
  DrakeShapes::VisualElement visual(X_WV);
  Eigen::Vector3d box_size(1, 1, 1);
  visual.setGeometry(DrakeShapes::Box(box_size));
  const int kBodyID = 0;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);
  Render();

  VerifyOutliers();

  // Verifies inside the box.
  const int x = kInlier.x;
  const int y = kInlier.y;
  // Color
  CompareColor(color_.at(x, y), kDefaultVisualColor, 255u,
               kColorPixelTolerance);
  // Depth
  ASSERT_NEAR(depth_.at(x, y)[0], 2.f, kDepthTolerance);
  // Label
  ASSERT_EQ(label_.at(x, y)[0], kBodyID);
}

TEST_F(RgbdRendererVTKTest, SphereTest) {
  Init(X_WC_, true);

  // Sets up a sphere.
  Isometry3d X_WV = Isometry3d::Identity();
  X_WV.translation().z() = 0.5;
  DrakeShapes::VisualElement visual(X_WV);
  visual.setGeometry(DrakeShapes::Sphere(0.5));
  const int kBodyID = 0;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);
  Render();

  VerifyOutliers();

  // Verifies inside the sphere.
  const int x = kInlier.x;
  const int y = kInlier.y;
  // Color
  CompareColor(color_.at(x, y), kDefaultVisualColor, 255u,
               kColorPixelTolerance);
  // Depth
  ASSERT_NEAR(depth_.at(x, y)[0], 2.f, kDepthTolerance);
  // Label
  ASSERT_EQ(label_.at(x, y)[0], kBodyID);
}

TEST_F(RgbdRendererVTKTest, CylinderTest) {
  Init(X_WC_, true);

  // Sets up a sphere.
  Isometry3d X_WV = Isometry3d::Identity();
  X_WV.translation().z() = 0.6;
  DrakeShapes::VisualElement visual(X_WV);
  visual.setGeometry(DrakeShapes::Cylinder(0.2, 1.2));  // Radius and length.
  const int kBodyID = 1;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);
  Render();

  VerifyOutliers();

  // Verifies inside the cylinder.
  const int x = kInlier.x;
  const int y = kInlier.y;
  // Color
  CompareColor(color_.at(x, y), kDefaultVisualColor, 255u,
               kColorPixelTolerance);
  // Depth
  ASSERT_NEAR(depth_.at(x, y)[0], 1.8f, kDepthTolerance);
  // Label
  ASSERT_EQ(label_.at(x, y)[0], kBodyID);
}

TEST_F(RgbdRendererVTKTest, MeshTest) {
  Init(X_WC_, true);

  Isometry3d X_WV = Isometry3d::Identity();
  DrakeShapes::VisualElement visual(X_WV);
  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  visual.setGeometry(DrakeShapes::Mesh("", filename));
  const int kBodyID = 0;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);
  Render();

  VerifyOutliers();

  // Verifies inside the cylinder.
  const int x = kInlier.x;
  const int y = kInlier.y;
  // Color
  CompareColor(color_.at(x, y), ColorI({4u, 241u, 33u}), 255u,
               kColorPixelTolerance);
  // Depth
  ASSERT_NEAR(depth_.at(x, y)[0], 2., kDepthTolerance);
  // Label
  ASSERT_EQ(label_.at(x, y)[0], kBodyID);
}

// TODO(kunimatsu-tri) Move DepthImageToPointCloudConversionTest here from
// rgbd_camera_test.

}  // namespace test
}  // namespace sensors
}  // namespace systems
}  // namespace drake
