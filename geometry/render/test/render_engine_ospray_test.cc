#include "drake/geometry/render/render_engine_ospray.h"

#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render/camera_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransformd;
using math::RotationMatrixd;
using std::make_unique;
using std::unique_ptr;
using std::unordered_map;
using std::vector;
using systems::sensors::Color;
using systems::sensors::ColorI;
using systems::sensors::ColorD;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

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

// Background (sky) and terrain colors.
const ColorI kBgColor = {254u, 127u, 0u};
const ColorD kTerrainColorD{0., 0., 0.};
const ColorI kTerrainColorI{0, 0, 0};
// box.png contains a single pixel with the color (4, 241, 33). If the image
// changes, the expected color would likewise have to change.
const ColorI kTextureColor{4, 241, 33};

// Provide a default visual color for these tests -- it is intended to be
// different from the default color of the VTK render engine.
const ColorI kDefaultVisualColor = {229u, 229u, 229u};
// Distance between the camera and the z = 0 ground plane.
const float kDefaultDistance{3.f};

// Values to be used with the "centered shape" tests.
// The amount inset from the edge of the images to *still* expect ground plane
// values.
static constexpr int kInset{10};

// Holds `(x, y)` indices of the screen coordinate system where the ranges of
// `x` and `y` are [0, image_width) and [0, image_height) respectively.
struct ScreenCoord {
  int x{};
  int y{};
};

std::ostream& operator<<(std::ostream& out, const ScreenCoord& c) {
  out << "(" << c.x << ", " << c.y << ")";
  return out;
}

// Utility struct for doing color testing; provides three mechanisms for
// creating a common rgba color. We get colors from images (as a pointer to
// unsigned bytes, as a (ColorI, alpha) pair, and from a normalized color. It's
// nice to articulate tests without having to worry about those details.
struct RgbaColor {
  RgbaColor(const Color<int>& c, int alpha)
      : r(c.r), g(c.g), b(c.b), a(alpha) {}
  explicit RgbaColor(const uint8_t* p) : r(p[0]), g(p[1]), b(p[2]), a(p[3]) {}
  explicit RgbaColor(const Vector4d& norm_color)
      : r(static_cast<int>(norm_color(0) * 255)),
        g(static_cast<int>(norm_color(1) * 255)),
        b(static_cast<int>(norm_color(2) * 255)),
        a(static_cast<int>(norm_color(3) * 255)) {}
  int r;
  int g;
  int b;
  int a;
};

std::ostream& operator<<(std::ostream& out, const RgbaColor& c) {
  out << "(" << c.r << ", " << c.g << ", " << c.b << ", " << c.a << ")";
  return out;
}

// Tests color within tolerance.
bool IsColorNear(
    const RgbaColor& expected, const RgbaColor& tested,
    double tolerance = kColorPixelTolerance) {
  using std::abs;
  return (abs(expected.r - tested.r) < tolerance &&
      abs(expected.g - tested.g) < tolerance &&
      abs(expected.b - tested.b) < tolerance &&
      abs(expected.a - tested.a) < tolerance);
}

// Tests that the color in the given `image` located at screen coordinate `p`
// matches the `expected` color to within the given `tolerance`.
::testing::AssertionResult CompareColor(
    const RgbaColor& expected, const ImageRgba8U& image, const ScreenCoord& p,
    double tolerance = kColorPixelTolerance) {
  RgbaColor tested(image.at(p.x, p.y));
  if (IsColorNear(expected, tested, tolerance)) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "Expected: " << expected
                                       << " at " << p
                                       << ", tested: " << tested
                                       << " with tolerance: " << tolerance;
}

// This test suite facilitates a test with a ground plane and floating shape.
// The camera is positioned above the shape looking straight down. All
// of the images produced from these tests should have the following properties:
//   1. The shape is centered.
//   2. The ground plane fills the whole background (i.e., no background color
//      should be visible), except for noted exceptions.
//   3. The rendered shape should be smaller than the full image size with a
//      minimum number of pixels of ground plane between the shape and the edge
//      of the image. The minimum number of pixels is defined by kInset.
//
// The tests examine the rendered images and tests some discrete pixels, mapped
// to the image size (w, h):
//   1. A "center" pixel (x, y) such that x = w / 2 and y = h / 2.
//   2. Border pixels (xᵢ, yᵢ) which are pixels inset from each corner:
//      e.g., (i, i), (w - i - 1, i), (w - i - 1, h - i - 1), (i, h - i - 1),
//      for an inset value of `i` pixels.
class RenderEngineOsprayTest : public ::testing::Test {
 public:
  RenderEngineOsprayTest()
      : color_(kWidth, kHeight),
        // Looking straight down from kDefaultDistance meters above the ground.
        X_WC_(RotationMatrixd{AngleAxisd(M_PI, Vector3d::UnitY()) *
                              AngleAxisd(-M_PI_2, Vector3d::UnitZ())},
              {0, 0, kDefaultDistance}),
        geometry_id_(GeometryId::get_new_id()) {}

 protected:
  // Method to allow the normal case (render with the built-in renderer against
  // the default camera) to the member images with default window visibility.
  // This interface allows that to be completely reconfigured by the calling
  // test.
  void Render(RenderEngineOspray* renderer = nullptr,
              const DepthCameraProperties* camera_in = nullptr,
              ImageRgba8U* color_out = nullptr) {
    if (!renderer) renderer = renderer_.get();
    const DepthCameraProperties& camera = camera_in ? *camera_in : camera_;
    ImageRgba8U* color = color_out ? color_out : &color_;
    renderer->RenderColorImage(camera, kShowWindow, color);
  }

  // Confirms that all pixels in the member color image have the same value.
  void VerifyUniformColor(const ColorI& pixel, int alpha) {
    const RgbaColor test_color{pixel, alpha};
    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x) {
        ASSERT_TRUE(CompareColor(test_color, color_, ScreenCoord{x, y}));
      }
    }
  }

  // Compute the set of outliers for a given set of camera properties.
  static vector<ScreenCoord> GetOutliers(const CameraProperties& camera) {
    return vector<ScreenCoord>{
        {kInset, kInset},
        {kInset, camera.height - kInset - 1},
        {camera.width - kInset - 1, camera.height - kInset - 1},
        {camera.width - kInset - 1, kInset}};
  }

  // Compute the inlier for the given set of camera properties.
  static ScreenCoord GetInlier(const CameraProperties& camera) {
    return ScreenCoord{camera.width / 2, camera.height / 2};
  }

  // Verifies the "outlier" pixels for the given camera belong to the ground
  // plane. If images are provided, the given images will be tested, otherwise
  // the member images will be tested.
  void VerifyOutliers(const RenderEngineOspray& renderer,
                      const DepthCameraProperties& camera,
                      const char* name,
                      ImageRgba8U* color_in = nullptr) {
    ImageRgba8U& color = color_in ? *color_in : color_;

    for (const auto& screen_coord : GetOutliers(camera)) {
      EXPECT_TRUE(CompareColor(expected_outlier_color_, color, screen_coord))
                << "Color at: " << screen_coord << " for test: " << name;
    }
  }

  void SetUp() override {
    ResetExpectations();
  }

  // Tests that don't instantiate their own renderers should invoke this.
  void Init(const RigidTransformd& X_WR, bool add_terrain = false) {
    const Vector3d bg_rgb{
        kBgColor.r / 255., kBgColor.g / 255., kBgColor.b / 255.};
    RenderEngineOsprayParams params{
        OsprayMode::kRayTracer, {}, bg_rgb, 1, true};
    renderer_ = make_unique<RenderEngineOspray>(params);
    InitializeRenderer(X_WR, add_terrain, renderer_.get());
    // Ensure that we truly have a non-default color.
    EXPECT_FALSE(IsColorNear(
        RgbaColor(kDefaultVisualColor, 1.),
        RgbaColor(renderer_->default_diffuse())));
  }

  // Tests that instantiate their own renderers can initialize their renderers
  // with this method.
  void InitializeRenderer(const RigidTransformd& X_WR, bool add_terrain,
                          RenderEngineOspray* engine) {
    engine->UpdateViewpoint(X_WR);

    if (add_terrain) {
      PerceptionProperties material;
      material.AddProperty("label", "id", RenderLabel::kDontCare);
      material.AddProperty(
          "phong", "diffuse",
          Vector4d{kTerrainColorD.r, kTerrainColorD.g, kTerrainColorD.b, 1.0});
      engine->RegisterVisual(GeometryId::get_new_id(), HalfSpace(), material,
                             RigidTransformd::Identity(),
                             false /** needs update */);
    }
  }

  // Creates a simple perception properties set for fixed, known results.
  PerceptionProperties simple_material(bool use_texture = false) const {
    PerceptionProperties material;
    Vector4d color_n(default_color_.r / 255., default_color_.g / 255.,
                     default_color_.b / 255., default_color_.a / 255.);
    material.AddProperty("phong", "diffuse", color_n);
    if (use_texture) {
      material.AddProperty(
          "phong", "diffuse_map",
          FindResourceOrThrow(
              "drake/systems/sensors/test/models/meshes/box.png"));
    }
    return material;
  }

  // Resets all expected values to the initial, default values.
  void ResetExpectations() {
    expected_color_ = RgbaColor{kDefaultVisualColor, 255};
    expected_outlier_color_ = RgbaColor(kTerrainColorI, 255);
  }

  // Populates the given renderer with the sphere required for
  // PerformCenterShapeTest().
  void PopulateSphereTest(RenderEngineOspray* renderer,
                          bool use_texture = false) {
    Sphere sphere{0.5};
    renderer->RegisterVisual(geometry_id_, sphere, simple_material(use_texture),
                             RigidTransformd::Identity(),
                             true /* needs update */);
    RigidTransformd X_WV{Vector3d{0, 0, 0.5}};
    X_WV_.clear();
    X_WV_.insert({geometry_id_, X_WV});
    renderer->UpdatePoses(X_WV_);
  }

  // Performs the work to test the rendering with a shape centered in the
  // image. To pass, the renderer will have to have been populated with a
  // compatible shape and camera configuration (e.g., PopulateSphereTest()).
  void PerformCenterShapeTest(RenderEngineOspray* renderer,
                              const char* name,
                              const DepthCameraProperties* camera = nullptr) {
    const DepthCameraProperties& cam = camera ? *camera : camera_;
    // Can't use the member images in case the camera has been configured to a
    // different size than the default camera_ configuration.
    ImageRgba8U color(cam.width, cam.height);
    Render(renderer, &cam, &color);

    VerifyOutliers(*renderer, cam, name, &color);

    // Verifies inside the sphere.
    const ScreenCoord inlier = GetInlier(cam);
    EXPECT_TRUE(CompareColor(expected_color_, color, inlier))
              << "Color at: " << inlier << " for test: " << name;
  }

  RgbaColor expected_color_{kDefaultVisualColor, 255};
  RgbaColor expected_outlier_color_{kDefaultVisualColor, 255};
  RgbaColor default_color_{kDefaultVisualColor, 255};

  const DepthCameraProperties camera_ = {kWidth, kHeight, kFovY, "unused",
                                         kZNear, kZFar};

  ImageRgba8U color_;
  RigidTransformd X_WC_;
  GeometryId geometry_id_;

  // The pose of the sphere created in PopulateSphereTest().
  unordered_map<GeometryId, RigidTransformd> X_WV_;

  unique_ptr<RenderEngineOspray> renderer_;
};

// Tests an empty image -- confirms that it clears to the "empty" color -- no
// use of "inlier" or "outlier" pixel locations.
TEST_F(RenderEngineOsprayTest, NoBodyTest) {
  Init(RigidTransformd::Identity());
  Render();

  VerifyUniformColor(kBgColor, 0u);
}

// Confirm that the color image background color gets successfully configured.
TEST_F(RenderEngineOsprayTest, ControlBackgroundColor) {
  std::vector<ColorI> backgrounds{{10, 20, 30}, {128, 196, 255}, {255, 10, 40}};
  for (const auto& bg : backgrounds) {
    RenderEngineOsprayParams params{
        OsprayMode::kRayTracer,
        {},
        Vector3d{bg.r / 255., bg.g / 255., bg.b / 255.},
        1,
        true};
    RenderEngineOspray engine(params);
    Render(&engine);
    VerifyUniformColor(bg, 0u);
  }
}

// Tests an image with *only* terrain (perpendicular to the camera's forward
// direction) -- no use of "inlier" or "outlier" pixel locations.
TEST_F(RenderEngineOsprayTest, TerrainTest) {
  Init(X_WC_, true);
  const Vector3d p_WR = X_WC_.translation();

  // At two different distances.
  for (auto depth : std::array<float, 2>({{2.f, 4.9999f}})) {
    X_WC_.set_translation({p_WR(0), p_WR(1), depth});
    renderer_->UpdateViewpoint(X_WC_);
    Render();
    VerifyUniformColor(kTerrainColorI, 255u);
  }

  // Closer than kZNear.
  X_WC_.set_translation({p_WR(0), p_WR(1), kZNear - 1e-5});
  renderer_->UpdateViewpoint(X_WC_);
  Render();
  VerifyUniformColor(kTerrainColorI, 255u);

  // Farther than kZFar.
  X_WC_.set_translation({p_WR(0), p_WR(1), kZFar + 1e-3});
  renderer_->UpdateViewpoint(X_WC_);
  Render();
  VerifyUniformColor(kTerrainColorI, 255u);
}

// Creates a terrain and then positions the camera such that a horizon between
// terrain and sky appears -- no use of "inlier" or "outlier" pixel locations.
TEST_F(RenderEngineOsprayTest, HorizonTest) {
  // Camera at the origin, pointing in a direction parallel to the ground.
  RigidTransformd X_WR{RotationMatrixd{AngleAxisd(-M_PI_2, Vector3d::UnitX()) *
      AngleAxisd(M_PI_2, Vector3d::UnitY())}};
  Init(X_WR, true);

  // Returns y in [0, kHeight / 2], index of horizon location in image
  // coordinate system under two assumptions: 1) the ground plane is not clipped
  // by `kClippingPlaneFar`, 2) camera is located above the ground.
  auto CalcHorizon = [](double z) {
    const double kTerrainSize = 50.;
    const double kFocalLength = kHeight * 0.5 / std::tan(0.5 * kFovY);
    return 0.5 * kHeight + z / kTerrainSize * kFocalLength;
  };

  // Verifies v index of horizon at three different camera heights.
  const Vector3d p_WR = X_WR.translation();
  for (const double z : {2., 1., 0.5}) {
    X_WR.set_translation({p_WR(0), p_WR(1), z});
    renderer_->UpdateViewpoint(X_WR);
    Render();

    int actual_horizon{0};
    for (int y = 0; y < kHeight; ++y) {
      // Looking for the boundary between the sky and the ground.
      if ((static_cast<uint8_t>(kBgColor.r != color_.at(0, y)[0])) ||
          (static_cast<uint8_t>(kBgColor.g != color_.at(0, y)[1])) ||
          (static_cast<uint8_t>(kBgColor.b != color_.at(0, y)[2]))) {
        actual_horizon = y;
        break;
      }
    }

    const double expected_horizon = CalcHorizon(z);
    // Confirm that the horizon is within one pixel + epsilon of where we
    // expect it to be.
    ASSERT_NEAR(expected_horizon, actual_horizon, 1.001);
  }
}

// Performs the shape-centered-in-the-image test with a box.
TEST_F(RenderEngineOsprayTest, BoxTest) {
  for (const bool use_texture : {false, true}) {
    for (const double texture_scale : {1.0, 0.5}) {
      const bool texture_scaled = texture_scale != 1;
      // We only need to sample texture scale if we're *using* the texture.
      if (!use_texture && texture_scaled) continue;
      Init(X_WC_, true);

      // Sets up a box.
      // Use non-uniform dimensions. Can't make the dimensions too large,
      // otherwise the box will extend towards the image boundaries, occluding
      // the background (outlier) pixels which _will_ register the wrong color.
      Box box(1.999, 0.55, 0.75);
      const GeometryId id = GeometryId::get_new_id();

      // For the box, we'll use a special texture that will allow us to detect
      // tiling. The default VTK cube source tiles the texture based on the
      // size of the box. We confirm that doesn't actually happen. We test both
      // the untiled default behavior and the ability to scale the texture.
      PerceptionProperties props = simple_material(false);
      if (use_texture) {
        props.AddProperty(
            "phong", "diffuse_map",
            FindResourceOrThrow(
                "drake/geometry/render/test/diag_gradient.png"));
        if (texture_scaled) {
          props.AddProperty(
              "phong", "diffuse_scale", Vector2d{texture_scale, texture_scale});
        }
      }
      renderer_->RegisterVisual(id, box, props,
                                RigidTransformd::Identity(),
                                true /* needs update */);
      // Position the box so that one corner is in the center of the image. The
      // corner will report the expected depth, color, and label. Typically, the
      // center of the box would render to the center of the image, so putting
      // the corner of the box at the center of the image would require moving
      // the box half its dimension lengths. Moving it *exactly* that length
      // means that the center pixel would only partially be filled (the vertex
      // would be in the center). We want the whole pixel filled so we place the
      // vertex just *beyond* the center so that the whole pixel is filled by
      // box face. The z-value comes from taking the *rotated* box and making
      // sure the near face has the expected depth.
      RigidTransformd X_WV{
          RotationMatrixd{AngleAxisd(M_PI, Vector3d::UnitX())},
          Vector3d{-box.width() * 0.49, -box.depth() * 0.49, 0.625}};
      renderer_->UpdatePoses(
          unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});

      if (texture_scaled) {
        // If we've scaled the texture:
        //   1. use_texture is enabled.
        //   2. The color isn't the ideal kTextureColor, because we're somewhere
        //      else in the gradient texture.
        // The gradient texture is crafted such that it has the same texture
        // color as box.png in the corner positioned at the center of the image.
        // When we scale the image differently, we'll radically change the
        // color at that same corner.

        // TODO(SeanCurtis-TIR): This color differs *slightly* from the VTK
        //  render engine version. Figure out what is necessary to get them
        //  to converge.
        expected_color_ = RgbaColor(ColorI{134, 115, 14}, 255);
        // Quick proof that we're testing for a different color -- we're drawing
        // the red channel from our expected color.
        ASSERT_NE(kTextureColor.r, 134);
      } else {
        // Otherwise the expected is simply the texture color of box.png.
        expected_color_ =
            use_texture ? RgbaColor(kTextureColor, 255) : default_color_;
      }

      PerformCenterShapeTest(
          renderer_.get(),
          fmt::format("Box test - {}",
                      use_texture ?
                      (texture_scaled ? "scaled texture" : "unscaled texture") :
                      "diffuse color")
              .c_str());
    }
  }
}

// Performs the shape-centered-in-the-image test with a sphere.
TEST_F(RenderEngineOsprayTest, SphereTest) {
  for (const bool use_texture : {false, true}) {
    Init(X_WC_, true);
    PopulateSphereTest(renderer_.get(), use_texture);
    expected_color_ =
        use_texture ? RgbaColor(kTextureColor, 255) : default_color_;
    PerformCenterShapeTest(
        renderer_.get(),
        fmt::format("Sphere test {}", use_texture ? "textured" : "rgba")
            .c_str());
  }
}

// Performs the shape-centered-in-the-image test with a sphere.
TEST_F(RenderEngineOsprayTest, TransparentSphereTest) {
  RenderEngineOspray renderer;
  InitializeRenderer(X_WC_, true /* add terrain */, &renderer);
  const int int_alpha = 128;
  default_color_ = RgbaColor(kDefaultVisualColor, int_alpha);
  PopulateSphereTest(&renderer);
  Render(&renderer);

  // Note: With an alpha value of 128, the resulting color value at the inlier
  // should be 50% terrain color and 50% visual color. The resultant alpha will
  // always be a full 255 (because the background is a full 255).
  auto blend = [](const ColorI& c1, const ColorI& c2, double alpha) {
    int r = static_cast<int>(c1.r * alpha + (c2.r * (1 - alpha)));
    int g = static_cast<int>(c1.g * alpha + (c2.g * (1 - alpha)));
    int b = static_cast<int>(c1.b * alpha + (c2.b * (1 - alpha)));
    return ColorI{r, g, b};
  };
  const double linear_factor = int_alpha / 255.0;
  const RgbaColor expect_linear{
      blend(kDefaultVisualColor, kTerrainColorI, linear_factor), 255};

  const ScreenCoord inlier = GetInlier(camera_);
  EXPECT_TRUE(CompareColor(expect_linear, color_, inlier));
}

// Performs the shape-centered-in-the-image test with a capsule.
TEST_F(RenderEngineOsprayTest, CapsuleTest) {
  Init(X_WC_, true);

  // Sets up a capsule.
  const double radius = 0.15;
  const double length = 1.2;
  Capsule capsule(radius, length);
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, capsule, simple_material(),
                            RigidTransformd::Identity(),
                            true /* needs update */);
  // Position the top of the capsule to be 1 m above the terrain. Since the
  // middle of the capsule is positioned at the origin 0, the top of the
  // capsule is placed at half the length plus the radius, i.e. 1.2/2 + 0.15 =
  // 0.75. To reach a total of 1, we need to offset it by an additional 0.25.
  RigidTransformd X_WV{Vector3d{0, 0, 0.25}};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});

  PerformCenterShapeTest(renderer_.get(), "Capsule test");
}

// Performs a test with a capsule centered in the image but rotated
// perpendicularly such that the length of the capsule can be seen in the
// camera view (as opposed to a top-down view of its spherical side).
// |          ●●
// |         ●  ●
// |        ●    ●
// |________●____●__________
// |        ●    ●
// |        ●    ●
// |         ●  ●
// |          ●●
TEST_F(RenderEngineOsprayTest, CapsuleRotatedTest) {
  Init(X_WC_, true);

  // Sets up a capsule.
  const double radius = 0.15;
  const double length = 1.2;
  Capsule capsule(radius, length);
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, capsule, simple_material(),
                            RigidTransformd::Identity(),
                            true /* needs update */);

  // Position the capsule so that it lies along the x-axis where the highest
  // point on the barrel is at z = 1. Capsules are by default z-axis aligned
  // so we need to rotate it by 90 degrees. Since the radius of the capsule is
  // 0.15, we need to shift it by an additional 0.85 along the z-axis to reach
  // a total of 1.
  RigidTransformd X_WV{RotationMatrixd{AngleAxisd(M_PI / 2, Vector3d::UnitY())},
                       Vector3d{0, 0, 0.85}};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});

  Render(renderer_.get());

  const char* name = "Capsule rotated test";
  VerifyOutliers(*renderer_, camera_, name);

  // Verifies the inliers towards the ends of the capsule and ensures its
  // length attribute is respected as opposed to just its radius. This
  // distinguishes it from other shape tests, such as a sphere.
  const ScreenCoord inlier = GetInlier(camera_);
  const int offsets[2] = {kHeight / 4, -kHeight / 4};
  const int x = inlier.x;
  for (const int& offset : offsets) {
    const int y = inlier.y + offset;
    const ScreenCoord offset_inlier = {x, y};
    EXPECT_TRUE(CompareColor(expected_color_, color_, offset_inlier))
        << "Color at: " << offset_inlier << " for test: " << name;
  }
}

// Performs the shape-centered-in-the-image test with a cylinder.
TEST_F(RenderEngineOsprayTest, CylinderTest) {
  for (const bool use_texture : {false, true}) {
    Init(X_WC_, true);

    // Sets up a cylinder.
    Cylinder cylinder(0.2, 1.2);
    const GeometryId id = GeometryId::get_new_id();
    renderer_->RegisterVisual(id, cylinder, simple_material(use_texture),
                              RigidTransformd::Identity(),
                              true /* needs update */);
    // Position the top of the cylinder to be 1 m above the terrain.
    RigidTransformd X_WV{Vector3d{0, 0, 0.4}};
    renderer_->UpdatePoses(
        unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});

    expected_color_ =
        use_texture ? RgbaColor(kTextureColor, 255) : default_color_;
    PerformCenterShapeTest(renderer_.get(), "Cylinder test");
  }
}

// Performs the shape-centered-in-the-image test with an ellipsoid rotated
// three different ways for confirming each extent axis.
TEST_F(RenderEngineOsprayTest, EllipsoidTest) {
  Init(X_WC_, true);

  // Sets up an ellipsoid.
  const double a = 0.25;
  const double b = 0.4;
  const double c = 0.5;
  Ellipsoid ellipsoid(a, b, c);
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, ellipsoid, simple_material(),
                            RigidTransformd::Identity(),
                            true /* needs update */);

  const double target_z = 1.0;

  // By default the 'c' extent of the ellipsoid is aligned with the z-axis of
  // the world. For the test we need to align the top of the ellipsoid to be at
  // the target height above the terrain, so we move it by (target_z - c) units
  // along the z-axis.
  RigidTransformd X_WV{Vector3d{0, 0, target_z - c}};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});
  PerformCenterShapeTest(renderer_.get(), "Ellipsoid test: c extent");

  // Rotate the ellipsoid so that the 'b' extent is aligned with the z-axis of
  // the world, then move it by (target_z - b) units along the z-axis.
  X_WV =
      RigidTransformd{RotationMatrixd{AngleAxisd(-M_PI / 2, Vector3d::UnitX())},
                      Vector3d{0, 0, target_z - b}};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});
  PerformCenterShapeTest(renderer_.get(), "Ellipsoid test: b extent");

  // Rotate the ellipsoid so that the 'a' extent is aligned with the z-axis of
  // the world, then move it by (target_z - a) units along the z-axis.
  X_WV =
      RigidTransformd{RotationMatrixd{AngleAxisd(M_PI / 2, Vector3d::UnitY())},
                      Vector3d{0, 0, target_z - a}};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});
  PerformCenterShapeTest(renderer_.get(), "Ellipsoid test: a extent");
}

// Performs the shape-centered-in-the-image test with a mesh (which happens to
// be a box). This simultaneously confirms that if a diffuse_map is specified
// but it doesn't refer to a file that can be read, that the appearance defaults
// to the diffuse rgba value.
TEST_F(RenderEngineOsprayTest, MeshTest) {
  Init(X_WC_, true);

  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  Mesh mesh(filename);
  PerceptionProperties material = simple_material();
  material.AddProperty("phong", "diffuse_map", "bad_path");
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, mesh, material, RigidTransformd::Identity(),
                            true /* needs update */);
  renderer_->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
      {id, RigidTransformd::Identity()}});

  PerformCenterShapeTest(renderer_.get(), "Mesh test");
}

// Performs the shape-centered-in-the-image test with a *textured* mesh (which
// happens to be a box).
TEST_F(RenderEngineOsprayTest, TextureMeshTest) {
  Init(X_WC_, true);

  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  Mesh mesh(filename);
  PerceptionProperties material = simple_material();
  material.AddProperty(
      "phong", "diffuse_map",
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.png"));
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, mesh, material, RigidTransformd::Identity(),
                            true /* needs update */);
  renderer_->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
      {id, RigidTransformd::Identity()}});

  expected_color_ = RgbaColor(kTextureColor, 255);
  PerformCenterShapeTest(renderer_.get(), "Textured mesh test");

  // Now confirm that the texture survives cloning.
  unique_ptr<RenderEngine> clone = renderer_->Clone();
  EXPECT_NE(dynamic_cast<RenderEngineOspray*>(clone.get()), nullptr);
  PerformCenterShapeTest(dynamic_cast<RenderEngineOspray*>(clone.get()),
                         "Cloned mesh test");
}

// Repeat the texture test but with an *implied* texture map. In other words,
// registering a mesh "foo.obj" will look for a "foo.png" in the same folder as
// a fall back and use it if found. But *only* as a back up. This is a
// SHORT TERM hack to get textures in.
TEST_F(RenderEngineOsprayTest, ImpliedTextureMeshTest) {
  Init(X_WC_, true);

  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  Mesh mesh(filename);
  PerceptionProperties material = simple_material();
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, mesh, material, RigidTransformd::Identity(),
                            true /* needs update */);
  renderer_->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
      {id, RigidTransformd::Identity()}});

  expected_color_ = RgbaColor(kTextureColor, 255);
  PerformCenterShapeTest(renderer_.get(), "Implied textured mesh test");
}

// This confirms that geometries are correctly removed from the render engine.
// We add two new geometries (testing the rendering after each addition).
// We remove the first of the added geometries -- because it's occluded, there
// should be no image change. Then by removing the second we should restore
// the original default image.
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
TEST_F(RenderEngineOsprayTest, RemoveVisual) {
  Init(X_WC_, true);
  PopulateSphereTest(renderer_.get());
  RgbaColor default_color = expected_color_;

  // Positions a sphere centered at <0, 0, z> with the given color.
  auto add_sphere = [this](const RgbaColor& diffuse, double z,
                           GeometryId geometry_id) {
    const double kRadius = 0.5;
    Sphere sphere{kRadius};
    Vector4d norm_diffuse{diffuse.r / 255., diffuse.g / 255., diffuse.b / 255.,
                          diffuse.a / 255.};
    PerceptionProperties material;
    material.AddProperty("phong", "diffuse", norm_diffuse);

    renderer_->RegisterVisual(geometry_id, sphere, material,
                              RigidTransformd::Identity());
    RigidTransformd X_WV{Vector3d{0, 0, z}};
    X_WV_.insert({geometry_id, X_WV});
    renderer_->UpdatePoses(X_WV_);
  };

  // Sets the expected values prior to calling PerformCenterShapeTest().
  auto set_expectations = [this](const RgbaColor& color) {
    expected_color_ = color;
  };

  // Add another sphere of a different color in front of the default sphere
  const RgbaColor color1(Color<int>{128, 128, 255}, 255);
  const GeometryId id1 = GeometryId::get_new_id();
  add_sphere(color1, 0.75, id1);
  set_expectations(color1);
  PerformCenterShapeTest(renderer_.get(), "First sphere added in remove test");

  // Add a _third_ sphere in front of the second.
  const RgbaColor color2(Color<int>{128, 255, 128}, 255);
  const GeometryId id2 = GeometryId::get_new_id();
  add_sphere(color2, 1.0, id2);
  set_expectations(color2);
  PerformCenterShapeTest(renderer_.get(), "Second sphere added in remove test");

  // Remove the first sphere added; should report "true" and the render test
  // should pass without changing expectations.
  bool removed = renderer_->RemoveGeometry(id1);
  EXPECT_TRUE(removed);
  PerformCenterShapeTest(renderer_.get(), "First added sphere removed");

  // Remove the second added sphere; should report true and rendering should
  // return to its default configuration.
  removed = renderer_->RemoveGeometry(id2);
  EXPECT_TRUE(removed);
  set_expectations(default_color);
  PerformCenterShapeTest(renderer_.get(),
                         "Default image restored by removing extra geometries");
}

// All of the clone tests use the PerformCenterShapeTest() with the sphere setup
// to confirm that the clone is behaving as anticipated.

// Tests that the cloned renderer produces the same images (i.e., passes the
// same test).
TEST_F(RenderEngineOsprayTest, SimpleClone) {
  Init(X_WC_, true);
  PopulateSphereTest(renderer_.get());
  PerformCenterShapeTest(renderer_.get(), "base_case");

  unique_ptr<RenderEngine> clone = renderer_->Clone();
  EXPECT_NE(dynamic_cast<RenderEngineOspray*>(clone.get()), nullptr);
  PerformCenterShapeTest(static_cast<RenderEngineOspray*>(clone.get()),
                         "Simple clone");
}

// Tests that the cloned renderer still works, even when the original is
// deleted.
TEST_F(RenderEngineOsprayTest, ClonePersistence) {
  Init(X_WC_, true);
  PopulateSphereTest(renderer_.get());

  unique_ptr<RenderEngine> clone = renderer_->Clone();
  // This causes the original renderer copied from to be destroyed.
  renderer_.reset();
  ASSERT_EQ(nullptr, renderer_);
  PerformCenterShapeTest(static_cast<RenderEngineOspray*>(clone.get()),
                         "Clone persistence");
}

// Tests that the cloned renderer still works, even when the original has values
// changed.
TEST_F(RenderEngineOsprayTest, CloneIndependence) {
  Init(X_WC_, true);
  PopulateSphereTest(renderer_.get());

  unique_ptr<RenderEngine> clone = renderer_->Clone();
  // Move the terrain *up* 10 units in the z.
  RigidTransformd X_WT_new{Vector3d{0, 0, 10}};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{geometry_id_, X_WT_new}});
  PerformCenterShapeTest(static_cast<RenderEngineOspray*>(clone.get()),
                         "Clone independence");
}

// Confirm that the renderer can be used for cameras with different properties.
// I.e., the camera intrinsics are defined *outside* the renderer.
TEST_F(RenderEngineOsprayTest, DifferentCameras) {
  Init(X_WC_, true);
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
}

// Confirms that efforts to render depth or label images throw.
TEST_F(RenderEngineOsprayTest, UnsupportedLabelAndDepth) {
  Init(X_WC_, true);

  ImageDepth32F depth(camera_.width, camera_.height);
  ImageLabel16I label(camera_.width, camera_.height);

  DRAKE_EXPECT_THROWS_MESSAGE(
      renderer_->RenderDepthImage(camera_, &depth), std::runtime_error,
      "RenderEngineOspray does not support depth images");
  DRAKE_EXPECT_THROWS_MESSAGE(
      renderer_->RenderLabelImage(camera_, false, &label), std::runtime_error,
      "RenderEngineOspray does not support label images");
}

// TODO(SeanCurtis-TRI): When we have a denoiser available, test this against
//  the pathtracer configuration and samples value.

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
