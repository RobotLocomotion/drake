#include "drake/geometry/render/render_engine_vtk.h"

#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;

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
class RenderEngineVtkTest : public ::testing::Test {
 public:
  RenderEngineVtkTest()
      : color_(kWidth, kHeight),
        depth_(kWidth, kHeight),
        label_(kWidth, kHeight),
        // Looking strait down from 3m above the ground.
        X_WC_(RotationMatrixd{AngleAxisd(M_PI, Vector3d::UnitY()) *
                              AngleAxisd(-M_PI_2, Vector3d::UnitZ())},
              {0, 0, kDefaultDistance}) {}

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
  void Init(const RigidTransformd& X_WC, bool add_terrain = false) {
    renderer_ = std::make_unique<Renderer>(
        RenderingConfig{kWidth, kHeight, kFovY, kZNear, kZFar, kShowWindow},
        X_WC);

    if (add_terrain) renderer_->AddFlatTerrain();
  }

  // Performs the work to test the rendering with a sphere centered in the
  // image. To pass, the renderer will have to have been populated with a
  // compliant sphere and camera configuration (e.g., PopulateSphereTest()).
  // If force_hidden is true, then the render windows will be suppressed
  // regardless of any other settings.
  void PerformCenterShapeTest() {
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
    // Label -- note: in this commit, kBodyID is *not* defined.
    ASSERT_EQ(label_.at(x, y)[0], kBodyID);
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
  RigidTransformd X_WC_;

  std::unique_ptr<RgbdRenderer> renderer_;
};

TEST_F(RenderEngineVtkTest, InstantiationTest) {
  Init(RigidTransformd::Identity());

  EXPECT_EQ(renderer_->config().width, kWidth);
  EXPECT_EQ(renderer_->config().height, kHeight);
  EXPECT_EQ(renderer_->config().fov_y, kFovY);
}

TEST_F(RenderEngineVtkTest, NoBodyTest) {
  Init(RigidTransformd::Identity());
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

TEST_F(RenderEngineVtkTest, TerrainTest) {
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

TEST_F(RenderEngineVtkTest, HorizonTest) {
  // Camera at the origin, pointing in a direction parallel to the ground.
  RigidTransformd X_WC{RotationMatrixd{AngleAxisd(-M_PI_2, Vector3d::UnitX()) *
                                       AngleAxisd(M_PI_2, Vector3d::UnitY())}};
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

TEST_F(RenderEngineVtkTest, BoxTest) {
  Init(X_WC_, true);

  // Sets up a box.
  RigidTransformd X_WV = RigidTransformd::Identity();
  X_WV.translation().z() = 0.5;
  DrakeShapes::VisualElement visual(X_WV);
  Eigen::Vector3d box_size(1, 1, 1);
  visual.setGeometry(DrakeShapes::Box(box_size));
  const int kBodyID = 0;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);

  PerformCenterShapeTest();
}

TEST_F(RenderEngineVtkTest, SphereTest) {
  Init(X_WC_, true);

  // Sets up a sphere.
  RigidTransformd X_WV = RigidTransformd::Identity();
  X_WV.translation().z() = 0.5;
  DrakeShapes::VisualElement visual(X_WV);
  visual.setGeometry(DrakeShapes::Sphere(0.5));
  const int kBodyID = 0;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);

  PerformCenterShapeTest();
}

TEST_F(RenderEngineVtkTest, CylinderTest) {
  Init(X_WC_, true);

  // Sets up a sphere.
  RigidTransformd X_WV = RigidTransformd::Identity();
  X_WV.translation().z() = 0.6;
  DrakeShapes::VisualElement visual(X_WV);
  visual.setGeometry(DrakeShapes::Cylinder(0.2, 1.2));  // Radius and length.
  const int kBodyID = 1;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);

  PerformCenterShapeTest();
}

TEST_F(RenderEngineVtkTest, MeshTest) {
  Init(X_WC_, true);

  RigidTransformd X_WV = RigidTransformd::Identity();
  DrakeShapes::VisualElement visual(X_WV);
  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  visual.setGeometry(DrakeShapes::Mesh("", filename));
  const int kBodyID = 0;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);

  PerformCenterShapeTest();
}

// TODO(kunimatsu-tri) Move DepthImageToPointCloudConversionTest here from
// rgbd_camera_test.

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
