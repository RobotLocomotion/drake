#include "drake/systems/sensors/rgbd_renderer_ospray.h"

#include "drake/systems/sensors/test/rgbd_renderer_test_util.h"

namespace drake {
namespace systems {
namespace sensors {
namespace test {

using RgbdRendererOSPRayTest = RgbdRendererTest<RgbdRendererOSPRay>;
using Eigen::Isometry3d;

TEST_F(RgbdRendererOSPRayTest, InstantiationTest) {
  Init(Isometry3d::Identity());

  EXPECT_EQ(renderer_->config().width, kWidth);
  EXPECT_EQ(renderer_->config().height, kHeight);
  EXPECT_EQ(renderer_->config().fov_y, kFovY);
}

TEST_F(RgbdRendererOSPRayTest, NoBodyTest) {
  // Rotates so that the light will be out of the field of view.
  const Isometry3d X_WC = Eigen::Translation3d(0, 0, 0) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());

  Init(X_WC);
  renderer_->RenderColorImage(&color_);
  const ColorI kNoBody({0, 0, 0});
  VerifyUniformColor(kNoBody, 0u);
}

TEST_F(RgbdRendererOSPRayTest, TerrainTest) {
  Init(X_WC_, true);

  const ColorI kTerrain({255, 255, 255});
  // At two different distances.
  for (auto depth : std::array<float, 2>({{2.f, 4.9999f}})) {
    X_WC_.translation().z() = depth;
    renderer_->UpdateViewpoint(X_WC_);
    renderer_->RenderColorImage(&color_);
    VerifyUniformColor(kTerrain, 254u);
  }

  // Closer than kZNear.
  X_WC_.translation().z() = kZNear - 1e-5;
  renderer_->UpdateViewpoint(X_WC_);
  renderer_->RenderColorImage(&color_);
  VerifyUniformColor(kTerrain, 254u);

  // Farther than kZFar.
  X_WC_.translation().z() = kZFar + 1e-5;
  renderer_->UpdateViewpoint(X_WC_);
  renderer_->RenderColorImage(&color_);
  VerifyUniformColor(kTerrain, 254u);
}

TEST_F(RgbdRendererOSPRayTest, HorizonTest) {
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
  const auto kSky = ColorI({0, 0, 0});
  const std::array<double, 3> Zs{{2., 1., 0.5}};
  for (const auto& z : Zs) {
    X_WC.translation().z() = z;
    renderer_->UpdateViewpoint(X_WC);
    renderer_->RenderColorImage(&color_);

    int actual_horizon{0};
    for (int y = 0; y < kHeight; ++y) {
      // Looking for the boundary between the sky and the ground.
      if ((static_cast<uint8_t>(kSky.r == color_.at(0, y)[0])) &&
          (static_cast<uint8_t>(kSky.g == color_.at(0, y)[1])) &&
          (static_cast<uint8_t>(kSky.b == color_.at(0, y)[2]))) {
        actual_horizon = y + 1;
      }
    }

    const double expected_horizon = CalcHorizon(z, kFovY, kHeight);
    ASSERT_NEAR(expected_horizon, actual_horizon, 1.001);
  }
}

TEST_F(RgbdRendererOSPRayTest, BoxTest) {
  Init(X_WC_, false);

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
  renderer_->RenderColorImage(&color_);

  // Verifies outside the box.
  for (const auto& p : kOutliers) {
    CompareColor(color_.at(p.x, p.y), ColorI({0, 0, 0}), 0,
                 kColorPixelTolerance);
  }
  // Verifies inside the box.
  const int x = kInlier.x;
  const int y = kInlier.y;
  // Color
  CompareColor(color_.at(x, y), ColorI({255, 255, 255}), 254u,
               kColorPixelTolerance);
}

TEST_F(RgbdRendererOSPRayTest, SphereTest) {
  Init(X_WC_, false);

  // Sets up a sphere.
  Isometry3d X_WV = Isometry3d::Identity();
  X_WV.translation().z() = 0.5;
  DrakeShapes::VisualElement visual(X_WV);
  visual.setGeometry(DrakeShapes::Sphere(0.5));
  const int kBodyID = 0;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);
  renderer_->RenderColorImage(&color_);

  // Verifies outside the sphere.
  for (const auto& p : kOutliers) {
    CompareColor(color_.at(p.x, p.y), ColorI({0, 0, 0}), 0,
                 kColorPixelTolerance);
  }
  // Verifies inside the sphere.
  const int x = kInlier.x;
  const int y = kInlier.y;
  // Color
  CompareColor(color_.at(x, y), ColorI({255, 255, 255}), 254u,
               kColorPixelTolerance);
}

TEST_F(RgbdRendererOSPRayTest, CylinderTest) {
  Init(X_WC_, false);

  // Sets up a cylinder.
  Isometry3d X_WV = Isometry3d::Identity();
  X_WV.translation().z() = 0.6;
  DrakeShapes::VisualElement visual(X_WV);
  visual.setGeometry(DrakeShapes::Cylinder(0.2, 1.2));  // Radius and length.
  const int kBodyID = 1;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);
  renderer_->RenderColorImage(&color_);

  // Verifies outside the cylinder.
  for (const auto& p : kOutliers) {
    CompareColor(color_.at(p.x, p.y), ColorI({0, 0, 0}), 0,
                 kColorPixelTolerance);
  }
  // Verifies inside the cylinder.
  const int x = kInlier.x;
  const int y = kInlier.y;
  CompareColor(color_.at(x, y), ColorI({255, 255, 255}), 254u,
               kColorPixelTolerance);
}

TEST_F(RgbdRendererOSPRayTest, MeshTest) {
  Init(X_WC_, false);

  Isometry3d X_WV = Isometry3d::Identity();
  DrakeShapes::VisualElement visual(X_WV);
  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  visual.setGeometry(DrakeShapes::Mesh("", filename));
  const int kBodyID = 0;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);
  renderer_->RenderColorImage(&color_);

  // Verifies outside the mesh.
  for (const auto& p : kOutliers) {
    CompareColor(color_.at(p.x, p.y), ColorI({0, 0, 0}), 0,
                 kColorPixelTolerance);
  }
  // Verifies inside the mesh.
  const int x = kInlier.x;
  const int y = kInlier.y;
  // Color
  CompareColor(color_.at(x, y), ColorI({104, 255, 129}), 214u, 2);
}

TEST_F(RgbdRendererOSPRayTest, MeshMaterialNotFoundTest) {
  Init(Isometry3d::Identity(), false);
  Isometry3d X_WV = Isometry3d::Identity();
  DrakeShapes::VisualElement visual(X_WV);
  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box2.obj");
  visual.setGeometry(DrakeShapes::Mesh("", filename));
  const int kBodyID = 0;
  const RgbdRenderer::VisualIndex kVisualID(0);
  EXPECT_THROW(renderer_->RegisterVisual(visual, kBodyID), std::runtime_error);
}

TEST_F(RgbdRendererOSPRayTest, SetBackgroundTest) {
  Init(X_WC_, false);
  auto* ospray = dynamic_cast<RgbdRendererOSPRay*>(renderer_.get());

  const char* kBackground = "drake/systems/sensors/test/models/background.jpg";
  auto file = FindResourceOrThrow(kBackground);
  EXPECT_NO_THROW(ospray->SetBackground(file));

  const char* kWrongFileFormat =
      "drake/systems/sensors/test/models/meshes/box.png";
  auto wrong_format = FindResourceOrThrow(kWrongFileFormat);
  EXPECT_THROW(ospray->SetBackground(wrong_format), std::runtime_error);

  const char* kNotExist =
      "drake/systems/sensors/test/models/meshes/no.png";
  EXPECT_THROW(ospray->SetBackground(kNotExist), std::runtime_error);
}

}  // namespace test
}  // namespace sensors
}  // namespace systems
}  // namespace drake
