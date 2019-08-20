#include "drake/geometry/render/render_engine_vtk.h"

#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render/camera_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/test_utilities/dummy_render_engine.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::internal::DummyRenderEngine;
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

// Background (sky) and terrain colors.
const ColorI kBgColor = {254u, 127u, 0u};
const ColorD kTerrainColorD{0., 0., 0.};
const ColorI kTerrainColorI{0, 0, 0};

// Provide a default visual color for these tests -- it is intended to be
// different from the default color of the VTK render engine.
const ColorI kDefaultVisualColor = {229u, 229u, 229u};
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
class RenderEngineVtkTest : public ::testing::Test {
 public:
  RenderEngineVtkTest()
      : color_(kWidth, kHeight),
        depth_(kWidth, kHeight),
        label_(kWidth, kHeight),
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
  void Render(RenderEngineVtk* renderer = nullptr,
              const DepthCameraProperties* camera_in = nullptr,
              ImageRgba8U* color_out = nullptr,
              ImageDepth32F* depth_out = nullptr,
              ImageLabel16I* label_out = nullptr) {
    if (!renderer) renderer = renderer_.get();
    const DepthCameraProperties& camera = camera_in ? *camera_in : camera_;
    ImageRgba8U* color = color_out ? color_out : &color_;
    ImageDepth32F* depth = depth_out ? depth_out : &depth_;
    ImageLabel16I* label = label_out ? label_out : &label_;
    renderer->RenderColorImage(camera, kShowWindow, color);
    renderer->RenderDepthImage(camera, depth);
    renderer->RenderLabelImage(camera, kShowWindow, label);
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

  // Confirms that all pixels in the member label image have the same value.
  void VerifyUniformLabel(int16_t label) {
    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x) {
        ASSERT_EQ(label_.at(x, y)[0], label)
                      << "At pixel (" << x << ", " << y << ")";
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

  // Tests that the depth value in the given `image` at the given `coord` is
  // the expected depth to within a tolerance. Handles the special case where
  // the expected distance is infinity.
  static ::testing::AssertionResult IsExpectedDepth(const ImageDepth32F& image,
                                                    const ScreenCoord& coord,
                                                    float expected_depth,
                                                    float tolerance) {
    const float actual_depth = image.at(coord.x, coord.y)[0];
    if (expected_depth == std::numeric_limits<float>::infinity()) {
      if (actual_depth == expected_depth) {
        return ::testing::AssertionSuccess();
      } else {
        return ::testing::AssertionFailure()
               << "Expected depth at " << coord << " to be infinity. Found: "
               << actual_depth;
      }
    } else {
      float delta = std::abs(expected_depth - actual_depth);
      if (delta <= tolerance) {
        return ::testing::AssertionSuccess();
      } else {
        return ::testing::AssertionFailure()
               << "Expected depth at " << coord << " to be " << expected_depth
               << ". Found " << actual_depth << ". Difference " << delta
               << " is greater than tolerance " << tolerance;
      }
    }
  }

  // Verifies the "outlier" pixels for the given camera belong to the ground
  // plane. If images are provided, the given images will be tested, otherwise
  // the member images will be tested.
  void VerifyOutliers(const RenderEngineVtk& renderer,
                      const DepthCameraProperties& camera,
                      const char* name,
                      ImageRgba8U* color_in = nullptr,
                      ImageDepth32F* depth_in = nullptr,
                      ImageLabel16I* label_in = nullptr) {
    ImageRgba8U& color = color_in ? *color_in : color_;
    ImageDepth32F& depth = depth_in ? *depth_in : depth_;
    ImageLabel16I& label = label_in ? *label_in : label_;

    for (const auto& screen_coord : GetOutliers(camera)) {
      const int x = screen_coord.x;
      const int y = screen_coord.y;
      EXPECT_TRUE(CompareColor(expected_outlier_color_, color, screen_coord))
                << "Color at: " << screen_coord << " for test: " << name;
      EXPECT_TRUE(IsExpectedDepth(depth, screen_coord, expected_outlier_depth_,
                                  kDepthTolerance))
                << "Depth at: " << screen_coord << " for test: " << name;
      EXPECT_EQ(label.at(x, y)[0], expected_outlier_label_)
                << "Label at: " << screen_coord << " for test: " << name;
    }
  }

  void SetUp() override {
    ResetExpectations();
  }

  // Tests that don't instantiate their own renderers should invoke this.
  void Init(const RigidTransformd& X_WR, bool add_terrain = false) {
    const Vector3d bg_rgb{
        kBgColor.r / 255., kBgColor.g / 255., kBgColor.b / 255.};
    RenderEngineVtkParams params{{}, {}, bg_rgb};
    renderer_ = make_unique<RenderEngineVtk>(params);
    InitializeRenderer(X_WR, add_terrain, renderer_.get());
    // Ensure that we truly have a non-default color.
    EXPECT_FALSE(IsColorNear(
        RgbaColor(kDefaultVisualColor, 1.),
        RgbaColor(renderer_->default_diffuse())));
  }

  // Tests that instantiate their own renderers can initialize their renderers
  // with this method.
  void InitializeRenderer(const RigidTransformd& X_WR, bool add_terrain,
                          RenderEngineVtk* engine) {
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

  // Creates a simple perception properties set for fixed, known results. The
  // material color can be modified by setting default_color_ prior to invoking
  // this method.
  PerceptionProperties simple_material() const {
    PerceptionProperties material;
    Vector4d color_n(default_color_.r / 255., default_color_.g / 255.,
                     default_color_.b / 255., default_color_.a / 255.);
    material.AddProperty("phong", "diffuse", color_n);
    material.AddProperty("label", "id", expected_label_);
    return material;
  }

  // Resets all expected values to the initial, default values.
  void ResetExpectations() {
    expected_color_ = RgbaColor{kDefaultVisualColor, 255};
    expected_outlier_color_ = RgbaColor(kTerrainColorI, 255);
    expected_outlier_depth_ = 3.f;
    expected_object_depth_ = 2.f;
    // We expect each test to explicitly set this.
    expected_label_ = RenderLabel();
    expected_outlier_label_ = RenderLabel::kDontCare;
  }

  // Populates the given renderer with the sphere required for
  // PerformCenterShapeTest().
  void PopulateSphereTest(RenderEngineVtk* renderer) {
    Sphere sphere{0.5};
    expected_label_ = RenderLabel(12345);  // an arbitrary value.
    renderer->RegisterVisual(geometry_id_, sphere, simple_material(),
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
    EXPECT_TRUE(CompareColor(expected_color_, color, inlier))
              << "Color at: " << inlier << " for test: " << name;
    EXPECT_TRUE(IsExpectedDepth(depth, inlier, expected_object_depth_,
                                kDepthTolerance))
              << "Depth at: " << inlier << " for test: " << name;
    EXPECT_EQ(label.at(x, y)[0], static_cast<int>(expected_label_))
              << "Label at: " << inlier << " for test: " << name;
  }

  RgbaColor expected_color_{kDefaultVisualColor, 255};
  RgbaColor expected_outlier_color_{kDefaultVisualColor, 255};
  float expected_outlier_depth_{3.f};
  float expected_object_depth_{2.f};
  RenderLabel expected_label_;
  RenderLabel expected_outlier_label_{RenderLabel::kDontCare};
  RgbaColor default_color_{kDefaultVisualColor, 255};

  const DepthCameraProperties camera_ = {kWidth, kHeight, kFovY, "unused",
                                         kZNear, kZFar};

  ImageRgba8U color_;
  ImageDepth32F depth_;
  ImageLabel16I label_;
  RigidTransformd X_WC_;
  GeometryId geometry_id_;

  // The pose of the sphere created in PopulateSphereTest().
  unordered_map<GeometryId, RigidTransformd> X_WV_;

  unique_ptr<RenderEngineVtk> renderer_;
};

// Tests an empty image -- confirms that it clears to the "empty" color -- no
// use of "inlier" or "outlier" pixel locations.
TEST_F(RenderEngineVtkTest, NoBodyTest) {
  Init(RigidTransformd::Identity());
  Render();

  VerifyUniformColor(kBgColor, 0u);
  VerifyUniformLabel(RenderLabel::kEmpty);
  VerifyUniformDepth(std::numeric_limits<float>::infinity());
}

// Confirm that the color image clear color gets successfully configured.
TEST_F(RenderEngineVtkTest, ControlBackgroundColor) {
  std::vector<ColorI> backgrounds{{10, 20, 30}, {128, 196, 255}, {255, 10, 40}};
  for (const auto& bg : backgrounds) {
    RenderEngineVtkParams params{
        {}, {}, Vector3d{bg.r / 255., bg.g / 255., bg.b / 255.}};
    RenderEngineVtk engine(params);
    Render(&engine);
    VerifyUniformColor(bg, 0u);
  }
}

// Tests an image with *only* terrain (perpendicular to the camera's forward
// direction) -- no use of "inlier" or "outlier" pixel locations.
TEST_F(RenderEngineVtkTest, TerrainTest) {
  Init(X_WC_, true);
  const Vector3d p_WR = X_WC_.translation();

  // At two different distances.
  for (auto depth : std::array<float, 2>({{2.f, 4.9999f}})) {
    X_WC_.set_translation({p_WR(0), p_WR(1), depth});
    renderer_->UpdateViewpoint(X_WC_);
    Render();
    VerifyUniformColor(kTerrainColorI, 255u);
    VerifyUniformLabel(RenderLabel::kDontCare);
    VerifyUniformDepth(depth);
  }

  // Closer than kZNear.
  X_WC_.set_translation({p_WR(0), p_WR(1), kZNear - 1e-5});
  renderer_->UpdateViewpoint(X_WC_);
  Render();
  VerifyUniformColor(kTerrainColorI, 255u);
  VerifyUniformLabel(RenderLabel::kDontCare);
  VerifyUniformDepth(InvalidDepth::kTooClose);

  // Farther than kZFar.
  X_WC_.set_translation({p_WR(0), p_WR(1), kZFar + 1e-3});
  renderer_->UpdateViewpoint(X_WC_);
  Render();
  VerifyUniformColor(kTerrainColorI, 255u);
  VerifyUniformLabel(RenderLabel::kDontCare);
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
    ASSERT_NEAR(expected_horizon, actual_horizon, 1.001);
  }
}

// Performs the shape-centered-in-the-image test with a box.
TEST_F(RenderEngineVtkTest, BoxTest) {
  Init(X_WC_, true);

  // Sets up a box.
  Box box(1, 1, 1);
  expected_label_ = RenderLabel(1);
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, box, simple_material(),
                            RigidTransformd::Identity(),
                            true /* needs update */);
  RigidTransformd X_WV{RotationMatrixd{AngleAxisd(M_PI, Vector3d::UnitX())},
                       Vector3d{0, 0, 0.5}};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});

  PerformCenterShapeTest(renderer_.get(), "Box test");
}

// Performs the shape-centered-in-the-image test with a sphere.
TEST_F(RenderEngineVtkTest, SphereTest) {
  Init(X_WC_, true);

  PopulateSphereTest(renderer_.get());

  PerformCenterShapeTest(renderer_.get(), "Sphere test");
}

// Performs the shape-centered-in-the-image test with a sphere.
TEST_F(RenderEngineVtkTest, TransparentSphereTest) {
  RenderEngineVtk renderer;
  InitializeRenderer(X_WC_, true /* add terrain */, &renderer);
  const int int_alpha = 128;
  default_color_ = RgbaColor(kDefaultVisualColor, int_alpha);
  PopulateSphereTest(&renderer);
  ImageRgba8U color(camera_.width, camera_.height);
  renderer.RenderColorImage(camera_, kShowWindow, &color);

  // Note: under CI this test runs with Xvfb - a virtual frame buffer. This does
  // *not* use the OpenGL drivers and, empirically, it has shown a different
  // alpha blending behavior.
  // For an alpha of 128 (i.e., 50%), the correct pixel color would be a
  // *linear* blend, i.e., 50% background and 50% foreground. However, the
  // implementation in Xvfb seems to be a function alpha *squared*. So, we
  // formulate this test to pass if the resultant pixel has one of two possible
  // colors.
  // In both cases, the resultant alpha will always be a full 255 (because the
  // background is a full 255).
  auto blend = [](const ColorI& c1, const ColorI& c2, double alpha) {
    int r = static_cast<int>(c1.r * alpha + (c2.r * (1 - alpha)));
    int g = static_cast<int>(c1.g * alpha + (c2.g * (1 - alpha)));
    int b = static_cast<int>(c1.b * alpha + (c2.b * (1 - alpha)));
    return ColorI{r, g, b};
  };
  const double linear_factor = int_alpha / 255.0;
  const RgbaColor expect_linear{
      blend(kDefaultVisualColor, kTerrainColorI, linear_factor), 255};
  const double quad_factor = linear_factor * (-linear_factor + 2);
  const RgbaColor expect_quad{
      blend(kDefaultVisualColor, kTerrainColorI, quad_factor), 255};

  const ScreenCoord inlier = GetInlier(camera_);
  EXPECT_TRUE(CompareColor(expect_linear, color, inlier) ||
              CompareColor(expect_quad, color, inlier));
}

// Performs the shape-centered-in-the-image test  with a cylinder.
TEST_F(RenderEngineVtkTest, CylinderTest) {
  Init(X_WC_, true);

  // Sets up a cylinder.
  Cylinder cylinder(0.2, 1.2);
  expected_label_ = RenderLabel(2);
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, cylinder, simple_material(),
                            RigidTransformd::Identity(),
                            true /* needs update */);
  // Position the top of the cylinder to be 1 m above the terrain.
  RigidTransformd X_WV{Vector3d{0, 0, 0.4}};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});

  PerformCenterShapeTest(renderer_.get(), "Cylinder test");
}

// Performs the shape-centered-in-the-image test with a mesh (which happens to
// be a box). This simultaneously confirms that if a diffuse_map is specified
// but it doesn't refer to a file that can be read, that the appearance defaults
// to the diffuse rgba value.
TEST_F(RenderEngineVtkTest, MeshTest) {
  Init(X_WC_, true);

  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  Mesh mesh(filename);
  expected_label_ = RenderLabel(3);
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
TEST_F(RenderEngineVtkTest, TextureMeshTest) {
  Init(X_WC_, true);

  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  Mesh mesh(filename);
  expected_label_ = RenderLabel(4);
  PerceptionProperties material = simple_material();
  material.AddProperty(
      "phong", "diffuse_map",
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.png"));
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, mesh, material, RigidTransformd::Identity(),
                            true /* needs update */);
  renderer_->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
      {id, RigidTransformd::Identity()}});

  // box.png contains a single pixel with the color (4, 241, 33). If the image
  // changes, the expected color would likewise have to change.
  expected_color_ = RgbaColor(ColorI{4, 241, 33}, 255);
  PerformCenterShapeTest(renderer_.get(), "Textured mesh test");

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
  Init(X_WC_, true);

  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  Mesh mesh(filename);
  expected_label_ = RenderLabel(4);
  PerceptionProperties material = simple_material();
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, mesh, material, RigidTransformd::Identity(),
                            true /* needs update */);
  renderer_->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
      {id, RigidTransformd::Identity()}});

  // box.png contains a single pixel with the color (4, 241, 33). If the image
  // changes, the expected color would likewise have to change.
  expected_color_ = RgbaColor(ColorI{4, 241, 33}, 255);
  PerformCenterShapeTest(renderer_.get(), "Implied textured mesh test");
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
  Init(X_WC_, true);
  PopulateSphereTest(renderer_.get());
  RgbaColor default_color = expected_color_;
  RenderLabel default_label = expected_label_;
  float default_depth = expected_object_depth_;

  // Positions a sphere centered at <0, 0, z> with the given color.
  auto add_sphere = [this](const RgbaColor& diffuse, double z,
                           GeometryId geometry_id) {
    const double kRadius = 0.5;
    Sphere sphere{kRadius};
    const float depth = kDefaultDistance - kRadius - z;
    Vector4d norm_diffuse{diffuse.r / 255., diffuse.g / 255., diffuse.b / 255.,
                          diffuse.a / 255.};
    RenderLabel label = RenderLabel(5);
    PerceptionProperties material;
    material.AddProperty("phong", "diffuse", norm_diffuse);
    material.AddProperty("label", "id", label);
    // This will accept all registered geometries and therefore, (bool)index
    // should always be true.
    renderer_->RegisterVisual(geometry_id, sphere, material,
                              RigidTransformd::Identity());
    RigidTransformd X_WV{Vector3d{0, 0, z}};
    X_WV_.insert({geometry_id, X_WV});
    renderer_->UpdatePoses(X_WV_);
    return std::make_tuple(label, depth);
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
  const GeometryId id1 = GeometryId::get_new_id();
  std::tie(label1, depth1) = add_sphere(color1, 0.75, id1);
  set_expectations(color1, depth1, label1);
  PerformCenterShapeTest(renderer_.get(), "First sphere added in remove test");

  // Add a _third_ sphere in front of the second.
  const RgbaColor color2(Color<int>{128, 255, 128}, 255);
  float depth2{};
  RenderLabel label2{};
  const GeometryId id2 = GeometryId::get_new_id();
  std::tie(label2, depth2) = add_sphere(color2, 1.0, id2);
  set_expectations(color2, depth2, label2);
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
  set_expectations(default_color, default_depth, default_label);
  PerformCenterShapeTest(renderer_.get(),
                         "Default image restored by removing extra geometries");
}

// All of the clone tests use the PerformCenterShapeTest() with the sphere setup
// to confirm that the clone is behaving as anticipated.

// Tests that the cloned renderer produces the same images (i.e., passes the
// same test).
TEST_F(RenderEngineVtkTest, SimpleClone) {
  Init(X_WC_, true);
  PopulateSphereTest(renderer_.get());
  PerformCenterShapeTest(renderer_.get(), "base_case");

  unique_ptr<RenderEngine> clone = renderer_->Clone();
  EXPECT_NE(dynamic_cast<RenderEngineVtk*>(clone.get()), nullptr);
  PerformCenterShapeTest(static_cast<RenderEngineVtk*>(clone.get()),
                         "Simple clone");
}

// Tests that the cloned renderer still works, even when the original is
// deleted.
TEST_F(RenderEngineVtkTest, ClonePersistence) {
  Init(X_WC_, true);
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
  Init(X_WC_, true);
  PopulateSphereTest(renderer_.get());

  unique_ptr<RenderEngine> clone = renderer_->Clone();
  // Move the terrain *up* 10 units in the z.
  RigidTransformd X_WT_new{Vector3d{0, 0, 10}};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{geometry_id_, X_WT_new}});
  PerformCenterShapeTest(static_cast<RenderEngineVtk*>(clone.get()),
                         "Clone independence");
}

// Confirm that the renderer can be used for cameras with different properties.
// I.e., the camera intrinsics are defined *outside* the renderer.
TEST_F(RenderEngineVtkTest, DifferentCameras) {
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

// Tests the ability to configure the RenderEngineVtk's default render label.
TEST_F(RenderEngineVtkTest, DefaultProperties_RenderLabel) {
  // A variation of PopulateSphereTest(), but uses an empty set of properties.
  // The result should be compatible with the running the sphere test.
  auto populate_default_sphere = [](auto* engine) {
    Sphere sphere{0.5};
    const GeometryId id = GeometryId::get_new_id();
    engine->RegisterVisual(id, sphere, PerceptionProperties(),
                           RigidTransformd::Identity(),
                           true /* needs update */);
    RigidTransformd X_WV{Vector3d{0, 0, 0.5}};
    engine->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});
  };

  // Case: No change to render engine's default must throw.
  {
    RenderEngineVtk renderer;
    InitializeRenderer(X_WC_, false /* no terrain */, &renderer);

    DRAKE_EXPECT_THROWS_MESSAGE(
        populate_default_sphere(&renderer),
        std::logic_error,
        ".* geometry with the 'unspecified' or 'empty' render labels.*");
  }

  // Case: Change render engine's default to explicitly be unspecified; must
  // throw.
  {
    RenderEngineVtk renderer{{RenderLabel::kUnspecified, {}}};
    InitializeRenderer(X_WC_, false /* no terrain */, &renderer);

    DRAKE_EXPECT_THROWS_MESSAGE(
        populate_default_sphere(&renderer),
        std::logic_error,
        ".* geometry with the 'unspecified' or 'empty' render labels.*");
  }

  // Case: Change render engine's default to don't care. Label image should
  // report don't care.
  {
    ResetExpectations();
    RenderEngineVtk renderer{{RenderLabel::kDontCare, {}}};
    InitializeRenderer(X_WC_, true /* no terrain */, &renderer);

    EXPECT_NO_THROW(populate_default_sphere(&renderer));
    expected_label_ = RenderLabel::kDontCare;
    expected_color_ = RgbaColor(renderer.default_diffuse());

    PerformCenterShapeTest(&renderer,
                           "Default properties; don't care label");
  }

  // Case: Change render engine's default to invalid default value; must throw.
  {
    for (RenderLabel label :
        {RenderLabel::kEmpty, RenderLabel(1), RenderLabel::kDoNotRender}) {
      DRAKE_EXPECT_THROWS_MESSAGE(
          RenderEngineVtk({label, {}}), std::logic_error,
          ".* default render label .* either 'kUnspecified' or 'kDontCare'.*");
    }
  }
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
