#include "drake/geometry/dev/render/render_engine_vtk.h"

#include <string>
#include <tuple>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/dev/render/camera_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace dev {
namespace render {
namespace {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector4d;
using std::make_unique;
using std::unique_ptr;
using systems::sensors::Color;
using systems::sensors::ColorI;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::InvalidDepth;

// Default camera properties.
const int kWidth = 640;
const int kHeight = 480;
const double kZNear = 0.5;
const double kZFar = 5.;
const double kFovY = M_PI_4;
const bool kShowWindow = false;

// The following tolerance is used due to a precision difference between Ubuntu
// Linux and Mac OSX.
const double kColorPixelTolerance = 1.001;
// NOTE: The depth tolerance is this large mostly due to the combination of
// several factors:
//   - the sphere test (sphere against terrain)
//   - The even-valued window dimensions
//   - the tests against various camera properties
// The explanation is as follows. The distance to the sphere is only exactly
// 2 at the point of the sphere directly underneath the camera (the sphere's
// "peak"). However, with an even-valued window dimension, we never really
// sample that point. We sample the center of pixels all evenly arrayed around
// that point. So, that introduces some error. As the image gets *smaller* the
// pixels get bigger and so the distance away from the peak center increases,
// which, in turn, increase the measured distance for the fragment. This
// tolerance accounts for the test case where one image has pixels that are *4X*
// larger (in area) than the default image size.
const double kDepthTolerance = 2e-4;

// Holds `(x, y)` indices of the screen coordinate system where the ranges of
// `x` and `y` are [0, image_width) and [0, image_height) respectively.
struct ScreenCoord {
  ScreenCoord(int x_in, int y_in) : x(x_in), y(y_in) {}
  int x{};
  int y{};
};

std::ostream& operator<<(std::ostream& out, const ScreenCoord& c) {
  out << "(" << c.x << ", " << c.y << ")";
  return out;
}

// Utility struct for doing color testing; provide two mechanisms for creating a
// common rgba color. We get colors from both images (as a pointer to unsigned
// bytes and as a (ColorI, alpha) pair. It's nice to articulate tests without
// having to worry about those details.
struct RgbaColor {
  RgbaColor(const Color<int>& c, int alpha)
      : r(c.r), g(c.g), b(c.b), a(alpha) {}
  explicit RgbaColor(const uint8_t* p) : r(p[0]), g(p[1]), b(p[2]), a(p[3]) {}
  int r;
  int g;
  int b;
  int a;
};

std::ostream& operator<<(std::ostream& out, const RgbaColor& c) {
  out << "(" << c.r << ", " << c.g << ", " << c.b << ", " << c.a << ")";
  return out;
}

// Tests that the color in the given `image` located at screen coordinate `p`
// matches the `expected` color to within the given `tolerance`.
::testing::AssertionResult CompareColor(
    const RgbaColor& expected, const ImageRgba8U& image, const ScreenCoord& p,
    double tolerance = kColorPixelTolerance) {
  using std::abs;
  RgbaColor tested(image.at(p.x, p.y));
  if (abs(expected.r - tested.r) < tolerance &&
      abs(expected.g - tested.g) < tolerance &&
      abs(expected.b - tested.b) < tolerance &&
      abs(expected.a - tested.a) < tolerance) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "Expected: " << expected
                                       << " at " << p
                                       << ", tested: " << tested
                                       << " with tolerance: " << tolerance;
}

// This suite tests RenderEngine. All of these tests introduce a ground plane
// with the terrain color and label and an *individual* shape floating above the
// mesh. The camera is positioned above the shape looking straight down. All
// of the images produced should have the following properties:
//   1. The shape is centered.
//   2. The terrain fills the whole background (i.e., no background color should
//      be visible), except for noted exceptions.
//   3. The rendered shape should be smaller than the full image size with a
//      minimum number of pixels of terrain between the shape and the edge of
//      the image. The minimum number of pixels is defined by kInset.
//
// The tests examine the rendered images and tests some discrete points, mapped
// to the image size (w, h):
//   1. Center point (x, y) such that x = w / 2 and y = h / 2.
//   2. Border points (xᵢ, yᵢ) which are fix pixels inset from each corner:
//      e.g., (i, i), (w - i, i), (w - i - 1, h - i - 1), (i, h - i - 1), for
//      an inset value of `i` pixels.
class RenderEngineVtkTest : public ::testing::Test {
 public:
  RenderEngineVtkTest()
      : color_(kWidth, kHeight),
        depth_(kWidth, kHeight),
        label_(kWidth, kHeight),
      // Looking straight down from 3m above the ground.
        X_WR_(Eigen::Translation3d(0, 0, kDefaultDistance) *
            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ())) {}

 protected:
  // Method to allow the normal case (render with the built-in renderer against
  // the default camera) to the member images with default window visibility.
  // This interface allows that to be completely reconfigured by the calling
  // test.
  void Render(RenderEngineVtk* renderer = nullptr,
              const DepthCameraProperties* camera_in = nullptr,
              ImageRgba8U* color_in = nullptr,
              ImageDepth32F* depth_in = nullptr,
              ImageLabel16I* label_in = nullptr) {
    if (!renderer) renderer = renderer_.get();
    const DepthCameraProperties& camera = camera_in ? *camera_in : camera_;
    ImageRgba8U* color = color_in ? color_in : &color_;
    ImageDepth32F* depth = depth_in ? depth_in : &depth_;
    ImageLabel16I* label = label_in ? label_in : &label_;
    renderer->RenderColorImage(camera, color, kShowWindow);
    renderer->RenderDepthImage(camera, depth);
    renderer->RenderLabelImage(camera, label, kShowWindow);
  }

  // Confirms that all pixels in the member color image have the same value.
  void VerifyUniformColor(const ColorI& pixel, int alpha) {
    const RgbaColor test_color{pixel, alpha};
    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x) {
        ASSERT_TRUE(CompareColor(test_color, color_, ScreenCoord(x,  y)));
      }
    }
  }

  // Confirms that all pixels in the member label image have the same value.
  void VerifyUniformLabel(int16_t label) {
    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x) {
        ASSERT_EQ(label_.at(x, y)[0], label);
      }
    }
  }

  // Confirms that all pixels in the member depth image have the same value.
  void VerifyUniformDepth(float depth) {
    if (depth == std::numeric_limits<float>::infinity()) {
      for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
          ASSERT_EQ(depth_.at(x, y)[0], depth);
        }
      }
    } else {
      for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
          ASSERT_NEAR(depth_.at(x, y)[0], depth, kDepthTolerance);
        }
      }
    }
  }

  // Compute the set of outliers for a given set of camera properties.
  static std::vector<ScreenCoord> GetOutliers(const CameraProperties& camera) {
    return std::vector<ScreenCoord>{
        {kInset, kInset},
        {kInset, camera.height - kInset - 1},
        {camera.width - kInset - 1, camera.height - kInset - 1},
        {camera.width - kInset - 1, kInset}};
  }

  // Compute the inlier for the given set of camera properties.
  static ScreenCoord GetInlier(const CameraProperties& camera) {
    return ScreenCoord(camera.width / 2, camera.height / 2);
  }

  // Tests that the depth value in the given `image` at the given `coord` is
  // the expected depth to within a tolerance. Handles the special case where
  // the expected distance is infinity.
  static ::testing::AssertionResult IsExpectedDepth(const ImageDepth32F& image,
  const ScreenCoord& coord, float expected_depth, float tolerance) {
    const float actual_depth = image.at(coord.x, coord.y)[0];
    if (expected_depth == std::numeric_limits<float>::infinity()) {
      if (actual_depth == expected_depth) {
        return ::testing::AssertionSuccess();
      } else {
        return ::testing::AssertionFailure()
            << "Expected depth at (" << coord.x << ", " << coord.y << ") to be "
            << "infinity. Found: " << actual_depth;
      }
    } else {
      float delta = std::abs(expected_depth - actual_depth);
      if (delta <= tolerance) {
        return ::testing::AssertionSuccess();
      } else {
        return ::testing::AssertionFailure()
            << "Expected depth at (" << coord.x << ", " << coord.y << ") to be "
            << expected_depth << ". Found " << actual_depth << ". Difference "
            << delta << "is greater than tolerance " << tolerance;
      }
    }
  }

  // Verifies the "outlier" pixels for the given camera belong to the terrain.
  // If images are provided, the given images will be tested, otherwise the
  // member images will be tested.
  void VerifyOutliers(const RenderEngineVtk& renderer,
                      const DepthCameraProperties& camera,
                      const char* name,
                      ImageRgba8U* color_in = nullptr,
                      ImageDepth32F* depth_in = nullptr,
                      ImageLabel16I* label_in = nullptr) {
    ImageRgba8U& color = color_in ? *color_in : color_;
    ImageDepth32F& depth = depth_in ? *depth_in : depth_;
    ImageLabel16I& label = label_in ? *label_in : label_;

    const auto& kTerrain = renderer.get_flat_terrain_color();
    for (const auto& screen_coord : GetOutliers(camera)) {
      const int x = screen_coord.x;
      const int y = screen_coord.y;
      // Color
      EXPECT_TRUE(CompareColor({kTerrain, 255}, color, screen_coord)) << name;
      // Depth
      EXPECT_TRUE(IsExpectedDepth(depth, screen_coord, expected_outlier_depth_,
                                  kDepthTolerance))<< name;
      // Label
      EXPECT_EQ(label.at(x, y)[0], RenderLabel::terrain_label()) << name;
    }
  }

  void SetUp() override {}

  // All tests on this class must invoke this first.
  void SetUp(const Eigen::Isometry3d& X_WR, bool add_terrain = false) {
    renderer_ = make_unique<RenderEngineVtk>();
    renderer_->UpdateViewpoint(X_WR);

    if (add_terrain) renderer_->AddFlatTerrain();
  }

  // Creates a simple perception properties set for fixed, known results.
  PerceptionProperties simple_material() const {
    PerceptionProperties material;
    material.AddGroup("phong");
    Vector4d color(kDefaultVisualColor.r / 255., kDefaultVisualColor.g / 255.,
                   kDefaultVisualColor.b / 255., 1.);
    material.AddProperty("phong", "diffuse", color);
    material.AddGroup("label");
    material.AddProperty("label", "id", expected_label_);
    return material;
  }

  // Populates the given renderer with the sphere required for
  // PerformCenterShapeTest().
  void PopulateSphereTest(RenderEngineVtk* renderer) {
    Sphere sphere{0.5};
    expected_label_ = RenderLabel::new_label();
    RenderIndex geometry_index = renderer->RegisterVisual(
        sphere, simple_material(), Isometry3d::Identity());
    Isometry3d X_WV{Eigen::Translation3d(0, 0, 0.5)};
    renderer->UpdateVisualPose(X_WV, geometry_index);
  }

  // Performs the work to test the rendering with a sphere centered in the
  // image. To pass, the renderer will have to have been populated with a
  // compliant sphere and camera configuration (e.g., PopulateSphereTest()).
  // If force_hidden is true, then the render windows will be suppressed
  // regardless of any other settings.
  void PerformCenterShapeTest(RenderEngineVtk* renderer,
                              const char* name,
                              const DepthCameraProperties* camera = nullptr) {
    const DepthCameraProperties& cam = camera ? *camera : camera_;
    // Can't use the member images in case the camera has been configured to a
    // different size than the default camera_ configuration.
    ImageRgba8U color(cam.width, cam.height);
    ImageDepth32F depth(cam.width, cam.height);
    ImageLabel16I label(cam.width, cam.height);
    Render(renderer, &cam, &color, &depth, &label);

    VerifyOutliers(*renderer, cam, name, &color, &depth, &label);

    // Verifies inside the sphere.
    const ScreenCoord inlier = GetInlier(cam);
    const int x = inlier.x;
    const int y = inlier.y;
    // Color
    EXPECT_TRUE(CompareColor(expected_color_, color, inlier)) << name;
    // Depth
    EXPECT_TRUE(IsExpectedDepth(depth, inlier, expected_object_depth_,
                                kDepthTolerance)) << name;
    // Label
    EXPECT_EQ(label.at(x, y)[0], static_cast<int>(expected_label_)) << name;
  }

  // Provide a default visual color for this tests -- it is intended to be
  // different from the default color of the VTK render engine.
  const ColorI kDefaultVisualColor = {229u, 229u, 229u};
  const float kDefaultDistance{3.f};

  // Values to be used with the "centered shape" tests.
  // The amount inset from the edge of the images to *still* expect terrain
  // values.
  static constexpr int kInset{10};
  RgbaColor expected_color_{kDefaultVisualColor, 255};
  float expected_outlier_depth_{3.f};
  float expected_object_depth_{2.f};
  RenderLabel expected_label_;

  const DepthCameraProperties camera_ = {kWidth, kHeight, kFovY, Fidelity::kLow,
                                         kZNear, kZFar};

  ImageRgba8U color_;
  ImageDepth32F depth_;
  ImageLabel16I label_;
  Isometry3d X_WR_;

  unique_ptr<RenderEngineVtk> renderer_;
};

// Tests an empty image -- confirms that it clears to the "empty" color -- no
// use of "inlier" or "outlier" pixel locations.
TEST_F(RenderEngineVtkTest, NoBodyTest) {
  SetUp(Isometry3d::Identity());
  Render();

  VerifyUniformColor(renderer_->get_sky_color(), 0u);
  VerifyUniformLabel(RenderLabel::empty_label());
  VerifyUniformDepth(std::numeric_limits<float>::infinity());
}

// Tests an image with *only* terrain (perpendicular to the camera's forward
// direction) -- no use of "inlier" or "outlier" pixel locations.
TEST_F(RenderEngineVtkTest, TerrainTest) {
  SetUp(X_WR_, true);

  const auto& kTerrain = renderer_->get_flat_terrain_color();
  // At two different distances.
  for (auto depth : std::array<float, 2>({{2.f, 4.9999f}})) {
    X_WR_.translation().z() = depth;
    renderer_->UpdateViewpoint(X_WR_);
    Render();
    VerifyUniformColor(kTerrain, 255u);
    VerifyUniformLabel(RenderLabel::terrain_label());
    VerifyUniformDepth(depth);
  }

  // Closer than kZNear.
  X_WR_.translation().z() = kZNear - 1e-5;
  renderer_->UpdateViewpoint(X_WR_);
  Render();
  VerifyUniformColor(kTerrain, 255u);
  VerifyUniformLabel(RenderLabel::terrain_label());
  VerifyUniformDepth(InvalidDepth::kTooClose);

  // Farther than kZFar.
  X_WR_.translation().z() = kZFar + 1e-3;
  renderer_->UpdateViewpoint(X_WR_);
  Render();
  VerifyUniformColor(kTerrain, 255u);
  VerifyUniformLabel(RenderLabel::terrain_label());
  // Verifies depth.
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      ASSERT_EQ(InvalidDepth::kTooFar, depth_.at(x, y)[0]);
    }
  }
}

// Creates a terrain and then positions the camera such that a horizon between
// terrain and sky appears -- no use of "inlier" or "outlier" pixel locations.
TEST_F(RenderEngineVtkTest, HorizonTest) {
  // Camera at the origin, pointing in a direction parallel to the ground.
  Isometry3d X_WR = Eigen::Translation3d(0, 0, 0) *
      Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
  SetUp(X_WR, true);

  // Returns y in [0, kHeight / 2], index of horizon location in image
  // coordinate system under two assumptions: 1) the ground plane is not clipped
  // by `kClippingPlaneFar`, 2) camera is located above the ground.
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

    const auto& kTerrain = renderer_->get_flat_terrain_color();
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

    const double expected_horizon = CalcHorizon(z);
    ASSERT_NEAR(expected_horizon, actual_horizon, 1.001);
  }
}

// Performs the shape centered in the image with a box.
TEST_F(RenderEngineVtkTest, BoxTest) {
  SetUp(X_WR_, true);

  // Sets up a box.
  Box box(1, 1, 1);
  expected_label_ = RenderLabel::new_label();
  RenderIndex geometry_index =
      renderer_->RegisterVisual(box, simple_material(), Isometry3d::Identity());
  Isometry3d X_WV{Eigen::Translation3d(0, 0, 0.5)};
  renderer_->UpdateVisualPose(X_WV, geometry_index);

  PerformCenterShapeTest(renderer_.get(), "Box test");
}

// Performs the shape centered in the image with a sphere.
TEST_F(RenderEngineVtkTest, SphereTest) {
  SetUp(X_WR_, true);

  PopulateSphereTest(renderer_.get());

  PerformCenterShapeTest(renderer_.get(), "Sphere test");
}

// Performs the shape centered in the image with a cylinder.
TEST_F(RenderEngineVtkTest, CylinderTest) {
  SetUp(X_WR_, true);

  // Sets up a cylinder.
  Cylinder cylinder(0.2, 1.2);
  expected_label_ = RenderLabel::new_label();
  RenderIndex geometry_index = renderer_->RegisterVisual(
      cylinder, simple_material(), Isometry3d::Identity());
  // Position the top of the cylinder to be 1 m above the terrain.
  Isometry3d X_WV{Eigen::Translation3d(0, 0, 0.4)};
  renderer_->UpdateVisualPose(X_WV, geometry_index);

  PerformCenterShapeTest(renderer_.get(), "Cylinder test");
}

// Performs the shape centered in the image with a mesh (which happens to be a
// box). This simultaneously confirms that if a diffuse_map is specified but it
// doesn't refer to a file that can be read, that the appearance defaults to
// the diffues rgba value.
TEST_F(RenderEngineVtkTest, MeshTest) {
  SetUp(X_WR_, true);

  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  Mesh mesh(filename);
  expected_label_ = RenderLabel::new_label();
  PerceptionProperties material = simple_material();
  // NOTE: Specifying a diffuse map with a known bad path, will force the box
  // to get the diffuse RGBA value (otherwise it would pick up the `box.png`
  // texture.
  material.AddProperty("phong", "diffuse_map", "bad_path");
  RenderIndex geometry_index = renderer_->RegisterVisual(
      mesh, material, Isometry3d::Identity());
  renderer_->UpdateVisualPose(Isometry3d::Identity(), geometry_index);

  PerformCenterShapeTest(renderer_.get(), "Mesh test");
}

// Performs the shape centered in the image with a *textured* mesh (which
// happens to be a box).
TEST_F(RenderEngineVtkTest, TextureMeshTest) {
  SetUp(X_WR_, true);

  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  Mesh mesh(filename);
  expected_label_ = RenderLabel::new_label();
  PerceptionProperties material = simple_material();
  material.AddProperty(
      "phong", "diffuse_map",
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.png"));
  RenderIndex geometry_index = renderer_->RegisterVisual(
      mesh, material, Isometry3d::Identity());
  renderer_->UpdateVisualPose(Isometry3d::Identity(), geometry_index);

  // box.png contains a single pixel with the color (4, 241, 33). If the image
  // changes, the expected color would likewise have to change.
  expected_color_ = RgbaColor(ColorI{4, 241, 33}, 255);
  PerformCenterShapeTest(renderer_.get(), "Mesh test");

  // Now confirm that the texture survives cloning.
  unique_ptr<RenderEngine> clone = renderer_->Clone();
  EXPECT_NE(dynamic_cast<RenderEngineVtk*>(clone.get()), nullptr);
  PerformCenterShapeTest(dynamic_cast<RenderEngineVtk*>(clone.get()),
                         "Cloned mesh test");
}

// Repeat the texture test but with an *implied* texture map. In other words,
// registering a mesh "foo.obj" will look for a "foo.png" in the same folder as
// a fall back and use it if found. But *only* as a back up. This is a
// SHORT TERM hack to get textures in.
TEST_F(RenderEngineVtkTest, ImpliedTextureMeshTest) {
  SetUp(X_WR_, true);

  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  Mesh mesh(filename);
  expected_label_ = RenderLabel::new_label();
  PerceptionProperties material = simple_material();
  RenderIndex geometry_index = renderer_->RegisterVisual(
      mesh, material, Isometry3d::Identity());
  renderer_->UpdateVisualPose(Isometry3d::Identity(), geometry_index);

  // box.png contains a single pixel with the color (4, 241, 33). If the image
  // changes, the expected color would likewise have to change.
  expected_color_ = RgbaColor(ColorI{4, 241, 33}, 255);
  PerformCenterShapeTest(renderer_.get(), "Mesh test");
}

// This confirms that geometries are correctly removed from the render engine.
// We add two new geometries (testing the rendering after each addition).
// By removing the first of the added geometries, we can confirm that the
// remaining geometries are re-ordered appropriately. Then by removing the,
// second we should restore the original default image.
//
// The default image is based on a sphere sitting on a plane at z = 0 with the
// camera located above the sphere's center and aimed at that center.
// THe default sphere is drawn with `●`, the first added sphere with `x`, and
// the second with `o`. The height of the top of each sphere and its depth in
// the camera's depth sensors are indicated as zᵢ and dᵢ, i ∈ {0, 1, 2},
// respectively.
//
//             /|\       <---- camera_z = 3
//              v
//
//
//
//
//            ooooo       <---- z₂ = 4r = 2, d₂ = 1
//          oo     oo
//         o         o
//        o           o
//        o           o
//        o   xxxxx   o   <---- z₁ = 3r = 1.5, d₁ = 1.5
//         oxx     xxo
//         xoo     oox
//        x   ooooo   x
//        x   ●●●●●   x   <---- z₀ = 2r = 1, d₀ = 2
//        x ●●     ●● x
//         ●         ●
//        ● xx     xx ●
// z      ●   xxxxx   ●
// ^      ●           ●
// |       ●         ●
// |        ●●     ●●
// |__________●●●●●____________
//
TEST_F(RenderEngineVtkTest, RemoveVisual) {
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());
  RgbaColor default_color = expected_color_;
  RenderLabel default_label = expected_label_;
  float default_depth = expected_object_depth_;

  // Positions a sphere centered at <0, 0, z> with the given color.
  auto add_sphere = [this](const RgbaColor& diffuse, double z) {
    const double kRadius = 0.5;
    Sphere sphere{kRadius};
    const float depth = kDefaultDistance - kRadius - z;
    Vector4d norm_diffuse{diffuse.r / 255., diffuse.g / 255., diffuse.b / 255.,
                          diffuse.a / 255.};
    RenderLabel label = RenderLabel::new_label();
    PerceptionProperties material;
    material.AddGroup("phong");
    material.AddProperty("phong", "diffuse", norm_diffuse);
    material.AddGroup("label");
    material.AddProperty("label", "id", label);
    RenderIndex index =
        renderer_->RegisterVisual(sphere, material, Isometry3d::Identity());
    Isometry3d X_WV{Eigen::Translation3d(0, 0, z)};
    renderer_->UpdateVisualPose(X_WV, index);
    return std::make_tuple(index, label, depth);
  };

  // Sets the expected values prior to calling PerformCenterShapeTest().
  auto set_expectations = [this](const RgbaColor& color, float depth,
                                 RenderLabel label) {
    expected_color_ = color;
    expected_label_ = label;
    expected_object_depth_ = depth;
  };

  // Add another sphere of a different color in front of the default sphere
  const RgbaColor color1(Color<int>{128, 128, 255}, 255);
  float depth1{};
  RenderLabel label1{};
  RenderIndex index1{};
  std::tie(index1, label1, depth1) = add_sphere(color1, 0.75);
  set_expectations(color1, depth1, label1);
  PerformCenterShapeTest(renderer_.get(), "First sphere added in remove test");

  // Add a _third_ sphere in front of the second.
  const RgbaColor color2(Color<int>{128, 255, 128}, 255);
  float depth2{};
  RenderLabel label2{};
  RenderIndex index2{};
  std::tie(index2, label2, depth2) = add_sphere(color2, 1.0);
  set_expectations(color2, depth2, label2);
  PerformCenterShapeTest(renderer_.get(), "Second sphere added in remove test");

  // Remove the first sphere added:
  //  1. index2 should be returned as the index of the shape that got moved.
  //  2. The test should pass without changing expectations.
  optional<RenderIndex> moved = renderer_->RemoveVisual(index1);
  EXPECT_TRUE(moved);
  EXPECT_EQ(*moved, index2);
  PerformCenterShapeTest(renderer_.get(), "First added sphere removed");

  // Remove the second added sphere (now indexed by index1):
  //  1. There should be no returned index.
  //  2. The rendering should match the default sphere test results.
  // Confirm restoration to original image.
  moved = nullopt;
  moved = renderer_->RemoveVisual(index1);
  EXPECT_FALSE(moved);
  set_expectations(default_color, default_depth, default_label);
  PerformCenterShapeTest(renderer_.get(),
                         "Default image restored by removing extra geometries");
}

// All of the clone tests use the PerformCenterShapeTest() with the sphere setup
// to confirm that the clone is behaving as anticipated.

// Tests that the cloned renderer produces the same images (i.e., passes the
// same test).
TEST_F(RenderEngineVtkTest, SimpleClone) {
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  unique_ptr<RenderEngine> clone = renderer_->Clone();
  EXPECT_NE(dynamic_cast<RenderEngineVtk*>(clone.get()), nullptr);
  PerformCenterShapeTest(dynamic_cast<RenderEngineVtk*>(clone.get()),
                         "Simple clone");
}

// Tests that the cloned renderer still works, even when the original is
// deleted.
TEST_F(RenderEngineVtkTest, ClonePersistence) {
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  unique_ptr<RenderEngine> clone = renderer_->Clone();
  // This causes the original renderer copied from to be destroyed.
  renderer_.reset();
  ASSERT_EQ(nullptr, renderer_);
  PerformCenterShapeTest(static_cast<RenderEngineVtk*>(clone.get()),
                         "Clone persistence");
}

// Tests that the cloned renderer still works, even when the original has values
// changed.
TEST_F(RenderEngineVtkTest, CloneIndependence) {
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  unique_ptr<RenderEngine> clone = renderer_->Clone();
  // Move the terrain *up* 10 units in the z.
  Isometry3d X_WT_new{Translation3d{0, 0, 10}};
  // This assumes that the terrain is zero-indexed.
  renderer_->UpdateVisualPose(X_WT_new, RenderIndex(0));
  PerformCenterShapeTest(dynamic_cast<RenderEngineVtk*>(clone.get()),
                         "Clone independence");
}

// Confirm that the renderer can be used for cameras with different properties.
// I.e., the camera intrinsics are defined *outside* the renderer.
TEST_F(RenderEngineVtkTest, DifferentCameras) {
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  // Baseline -- confirm that all of the defaults in this test still produce
  // the expected outcome.
  PerformCenterShapeTest(renderer_.get(), "Camera change - baseline", &camera_);

  // Test changes in sensor sizes.
  {
    // Now run it again with a camera with a smaller sensor (quarter area).
    DepthCameraProperties small_camera{camera_};
    small_camera.width /= 2;
    small_camera.height /= 2;
    PerformCenterShapeTest(renderer_.get(), "Camera change - small camera",
                           &small_camera);

    // Now run it again with a camera with a bigger sensor (4X area).
    DepthCameraProperties big_camera{camera_};
    big_camera.width *= 2;
    big_camera.height *= 2;
    PerformCenterShapeTest(renderer_.get(), "Camera change - big camera",
                           &big_camera);
  }

  // Test changes in fov (larger and smaller).
  {
    DepthCameraProperties narrow_fov(camera_);
    narrow_fov.fov_y /= 2;
    PerformCenterShapeTest(renderer_.get(), "Camera change - narrow fov",
                           &narrow_fov);

    DepthCameraProperties wide_fov(camera_);
    wide_fov.fov_y *= 2;
    PerformCenterShapeTest(renderer_.get(), "Camera change - wide fov",
                           &wide_fov);
  }

  // Test changes to depth range.
  {
    DepthCameraProperties clipping_far_plane(camera_);
    clipping_far_plane.z_far = expected_outlier_depth_ - 0.1;
    const float old_outlier_depth = expected_outlier_depth_;
    expected_outlier_depth_ = std::numeric_limits<float>::infinity();
    PerformCenterShapeTest(renderer_.get(),
                           "Camera change - z far clips terrain",
                           &clipping_far_plane);
    // NOTE: Need to restored expected outlier depth for next test.
    expected_outlier_depth_ = old_outlier_depth;

    DepthCameraProperties clipping_near_plane(camera_);
    clipping_near_plane.z_near = expected_object_depth_ + 0.1;
    expected_object_depth_ = 0;
    PerformCenterShapeTest(renderer_.get(),
                           "Camera change - z near clips mesh",
                           &clipping_near_plane);
  }
}

// Tests that registered geometry without specific values renders without error.
// TODO(SeanCurtis-TRI): When the ability to set defaults is exposed through a
// public API, actually test for the *default values*. Until then, error-free
// rendering is sufficient.
TEST_F(RenderEngineVtkTest, DefaultProperties) {
  SetUp(X_WR_, false  /* no terrain */);

  // Sets up a box.
  Box box(1, 1, 1);
  RenderIndex geometry_index = renderer_->RegisterVisual(
      box, PerceptionProperties(), Isometry3d::Identity());
  Isometry3d X_WV{Eigen::Translation3d(0, 0, 0.5)};
  renderer_->UpdateVisualPose(X_WV, geometry_index);

  EXPECT_NO_THROW(Render());
}

}  // namespace
}  // namespace render
}  // namespace dev
}  // namespace geometry
}  // namespace drake
