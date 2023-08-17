#include "drake/geometry/render_vtk/internal_render_engine_vtk.h"

#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <fmt/format.h>
#include <gtest/gtest.h>
#include <vtkOpenGLTexture.h>
#include <vtkPNGReader.h>
#include <vtkProperty.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransformd;
using math::RotationMatrixd;
using render::ColorRenderCamera;
using render::DepthRenderCamera;
using render::LightParameter;
using render::RenderCameraCore;
using render::RenderEngine;
using render::RenderLabel;
using std::make_unique;
using std::unique_ptr;
using std::unordered_map;
using std::vector;
using systems::sensors::CameraInfo;
using systems::sensors::Color;
using systems::sensors::ColorD;
using systems::sensors::ColorI;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::ImageTraits;
using systems::sensors::PixelType;

// Default camera properties.
const int kWidth = 640;
const int kHeight = 480;
const double kClipNear = 0.1;
const double kClipFar = 100.0;
const double kZNear = 0.5;
const double kZFar = 5.;
const double kFovY = M_PI_4;
/* Note: enabling this causes failures with two tests. Try running as:

 bazel test //geometry/render_vtk:internal_render_engine_vtk_test \
    --test_filter=-*DifferentCameras:*Intrinsics*

 to get past the aberrant tests; they *should* pass with this disabled. */
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
// that point. So, that introduces some error. This error is further increased
// in ellipsoid tests when sampling around the elongated ends. As the image gets
// *smaller* the pixels get bigger and so the distance away from the peak center
// increases, which, in turn, increase the measured distance for the fragment.
// This tolerance accounts for the test case where one image has pixels that are
// *4X* larger (in area) than the default image size.
const double kDepthTolerance = 1e-3;

// Background (sky) and terrain colors.
const ColorI kBgColor = {254u, 127u, 0u};
// We need a color that we can see the effects of illumination on.
const ColorD kTerrainColorD{0.498, 0.498, 0.6};
const ColorI kTerrainColorI{127, 127, 153};
// box.png contains a single pixel with the color (4, 241, 33). If the image
// changes, the expected color would likewise have to change.
const ColorI kTextureColor{4, 241, 33};

// Provide a default visual color for these tests -- it is intended to be
// different from the default color of the VTK render engine.
const ColorI kDefaultVisualColor = {229u, 229u, 229u};
const float kDefaultDistance{3.f};

const RenderLabel kDefaultLabel{13531};

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
  // We'll allow *implicit* conversion from Rgba to RgbaColor to increase the
  // utility of IsColorNear(), but only in the scope of this test.
  // NOLINTNEXTLINE(runtime/explicit)
  RgbaColor(const Rgba& rgba)
      : r(static_cast<int>(rgba.r() * 255)),
        g(static_cast<int>(rgba.g() * 255)),
        b(static_cast<int>(rgba.b() * 255)),
        a(static_cast<int>(rgba.a() * 255)) {}

  bool operator==(const RgbaColor& c) const {
    return r == c.r && g == c.g && b == c.b && a == c.a;
  }

  bool operator!=(const RgbaColor& c) const { return !(*this == c); }

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
bool IsColorNear(const RgbaColor& expected, const RgbaColor& tested,
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
  return ::testing::AssertionFailure()
         << "Expected: " << expected << " at " << p << ", tested: " << tested
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
              const DepthRenderCamera* camera_in = nullptr,
              ImageRgba8U* color_out = nullptr,
              ImageDepth32F* depth_out = nullptr,
              ImageLabel16I* label_out = nullptr) {
    if (!renderer) renderer = renderer_.get();
    const DepthRenderCamera& depth_camera =
        camera_in ? *camera_in : depth_camera_;
    const ColorRenderCamera color_camera(depth_camera.core(), kShowWindow);
    ImageRgba8U* color = color_out ? color_out : &color_;
    ImageDepth32F* depth = depth_out ? depth_out : &depth_;
    ImageLabel16I* label = label_out ? label_out : &label_;
    EXPECT_NO_THROW(renderer->RenderDepthImage(depth_camera, depth));
    EXPECT_NO_THROW(renderer->RenderLabelImage(color_camera, label));
    EXPECT_NO_THROW(renderer->RenderColorImage(color_camera, color));
  }

  // Confirms that all pixels in the member color image have the same value.
  void VerifyUniformColor(const ColorI& pixel, int alpha,
                          const ImageRgba8U* color = nullptr) {
    if (color == nullptr) color = &color_;
    const RgbaColor test_color{pixel, alpha};
    for (int y = 0; y < color->height(); ++y) {
      for (int x = 0; x < color->width(); ++x) {
        ASSERT_TRUE(CompareColor(test_color, *color, ScreenCoord{x, y}));
      }
    }
  }

  // Confirms that all pixels in the member label image have the same value.
  void VerifyUniformLabel(int16_t value, const ImageLabel16I* label = nullptr) {
    if (label == nullptr) label = &label_;
    for (int y = 0; y < label->height(); ++y) {
      for (int x = 0; x < label->width(); ++x) {
        ASSERT_EQ(label->at(x, y)[0], value)
            << "At pixel (" << x << ", " << y << ")";
      }
    }
  }

  // Confirms that all pixels in the member depth image have the same value.
  void VerifyUniformDepth(float value, const ImageDepth32F* depth = nullptr) {
    if (depth == nullptr) depth = &depth_;
    if (value == std::numeric_limits<float>::infinity()) {
      for (int y = 0; y < depth->height(); ++y) {
        for (int x = 0; x < depth->width(); ++x) {
          ASSERT_EQ(depth->at(x, y)[0], value);
        }
      }
    } else {
      for (int y = 0; y < depth->height(); ++y) {
        for (int x = 0; x < depth->width(); ++x) {
          ASSERT_NEAR(depth->at(x, y)[0], value, kDepthTolerance);
        }
      }
    }
  }

  // Compute the set of outliers for a given set of camera properties.
  static std::vector<ScreenCoord> GetOutliers(const CameraInfo& intrinsics) {
    return std::vector<ScreenCoord>{
        {kInset, kInset},
        {kInset, intrinsics.height() - kInset - 1},
        {intrinsics.width() - kInset - 1, intrinsics.height() - kInset - 1},
        {intrinsics.width() - kInset - 1, kInset}};
  }

  // Compute the inlier for the given set of camera properties.
  static ScreenCoord GetInlier(const CameraInfo& intrinsics) {
    return {intrinsics.width() / 2, intrinsics.height() / 2};
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
               << "Expected depth at " << coord
               << " to be infinity. Found: " << actual_depth;
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
                      const DepthRenderCamera& camera, const char* name,
                      const ImageRgba8U* color_in = nullptr,
                      const ImageDepth32F* depth_in = nullptr,
                      const ImageLabel16I* label_in = nullptr) const {
    const ImageRgba8U& color = color_in ? *color_in : color_;
    const ImageDepth32F& depth = depth_in ? *depth_in : depth_;
    const ImageLabel16I& label = label_in ? *label_in : label_;

    for (const auto& screen_coord : GetOutliers(camera.core().intrinsics())) {
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

  void SetUp() override { ResetExpectations(); }

  // Tests that don't instantiate their own renderers should invoke this.
  void Init(const RigidTransformd& X_WR, bool add_terrain = false) {
    const Vector3d bg_rgb{kBgColor.r / 255., kBgColor.g / 255.,
                          kBgColor.b / 255.};
    RenderEngineVtkParams params{{}, {}, bg_rgb};
    renderer_ = make_unique<RenderEngineVtk>(params);
    InitializeRenderer(X_WR, add_terrain, renderer_.get());
    // Ensure that we truly have a non-default color.
    EXPECT_FALSE(IsColorNear(RgbaColor(kDefaultVisualColor, 1.),
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
                             false /* needs update */);
    }
  }

  // Creates a simple perception properties set for fixed, known results. The
  // material color can be modified by setting default_color_ prior to invoking
  // this method.
  PerceptionProperties simple_material(bool use_texture = false) const {
    PerceptionProperties material;
    material.AddProperty("label", "id", expected_label_);
    if (use_texture) {
      // The simple material's texture should always reproduce the texture
      // perfectly -- so the diffuse color must be opaque white.
      material.AddProperty("phong", "diffuse", Rgba(1, 1, 1));

      material.AddProperty(
          "phong", "diffuse_map",
          FindResourceOrThrow("drake/geometry/render/test/meshes/box.png"));
    } else {
      const Rgba default_color(
          default_color_.r / 255.0, default_color_.g / 255.0,
          default_color_.b / 255.0, default_color_.a / 255.0);
      material.AddProperty("phong", "diffuse", default_color);
    }
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
  void PopulateSphereTest(RenderEngineVtk* renderer, bool use_texture = false) {
    Sphere sphere{0.5};
    expected_label_ = RenderLabel(12345);  // an arbitrary value.
    renderer->RegisterVisual(geometry_id_, sphere, simple_material(use_texture),
                             RigidTransformd::Identity(),
                             true /* needs update */);
    RigidTransformd X_WV{Vector3d{0, 0, 0.5}};
    X_WV_.clear();
    X_WV_.insert({geometry_id_, X_WV});
    renderer->UpdatePoses(X_WV_);
  }

  void PopulateSimpleBoxTest(RenderEngineVtk* renderer) {
    // Simple cube.
    const double length = 1.0;
    const Box box = Box::MakeCube(length);
    expected_label_ = kDefaultLabel;
    const GeometryId id = GeometryId::get_new_id();
    PerceptionProperties props = simple_material(false);
    renderer->RegisterVisual(id, box, props, RigidTransformd::Identity(),
                             true /* needs update */);
    // Leave the box centered on the xy plane, but raise it up for the expected
    // depth in the camera (distance from eye to near surface):
    //      expected depth = p_WC.z - length / 2 - p_WV.z;
    const double p_WVo_z =
        X_WC_.translation()(2) - length / 2 - expected_object_depth_;
    RigidTransformd X_WV{Vector3d{0, 0, p_WVo_z}};
    renderer->UpdatePoses(
        unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});
    expected_color_ = default_color_;
  }

  // Performs the work to test the rendering with a shape centered in the
  // image. To pass, the renderer will have to have been populated with a
  // compatible shape and camera configuration (e.g., PopulateSphereTest()).
  void PerformCenterShapeTest(RenderEngineVtk* renderer, const char* name,
                              const DepthRenderCamera* camera = nullptr) {
    const DepthRenderCamera& cam = camera ? *camera : depth_camera_;
    const int w = cam.core().intrinsics().width();
    const int h = cam.core().intrinsics().height();
    // Can't use the member images in case the camera has been configured to a
    // different size than the default camera_ configuration.
    ImageRgba8U color(w, h);
    ImageDepth32F depth(w, h);
    ImageLabel16I label(w, h);
    Render(renderer, &cam, &color, &depth, &label);

    VerifyCenterShapeTest(*renderer, name, cam, color, depth, label);
  }

  void VerifyCenterShapeTest(const RenderEngineVtk& renderer, const char* name,
                             const DepthRenderCamera& camera,
                             const ImageRgba8U& color,
                             const ImageDepth32F& depth,
                             const ImageLabel16I& label) const {
    VerifyOutliers(renderer, camera, name, &color, &depth, &label);

    // Verifies inside the sphere.
    const ScreenCoord inlier = GetInlier(camera.core().intrinsics());
    const int x = inlier.x;
    const int y = inlier.y;
    EXPECT_TRUE(CompareColor(expected_color_, color, inlier))
        << "Color at: " << inlier << " for test: " << name;
    EXPECT_TRUE(
        IsExpectedDepth(depth, inlier, expected_object_depth_, kDepthTolerance))
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

  // We store a reference depth camera; we can always derive a color camera
  // from it; they have the same intrinsics and we grab the global kShowWindow.
  const DepthRenderCamera depth_camera_{
      {"unused", {kWidth, kHeight, kFovY}, {kClipNear, kClipFar}, {}},
      {kZNear, kZFar}};

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

  VerifyUniformColor(kBgColor, 255u);
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
    VerifyUniformColor(bg, 255u);
  }
}

// Tests an image with *only* terrain (perpendicular to the camera's forward
// direction) -- no use of "inlier" or "outlier" pixel locations.
TEST_F(RenderEngineVtkTest, TerrainTest) {
  Init(X_WC_, true);
  const Vector3d p_WR = X_WC_.translation();

  // At several different distances.
  for (const float depth : {1.f, 2.f, 3.f, 4.f, 4.9999f}) {
    X_WC_.set_translation({p_WR(0), p_WR(1), depth});
    renderer_->UpdateViewpoint(X_WC_);
    Render();
    SCOPED_TRACE(fmt::format("Valid depth return: {}", depth));
    VerifyUniformColor(kTerrainColorI, 255u);
    VerifyUniformLabel(RenderLabel::kDontCare);
    VerifyUniformDepth(depth);
  }

  // Closer than kZNear.
  X_WC_.set_translation({p_WR(0), p_WR(1), kZNear - 1e-5});
  renderer_->UpdateViewpoint(X_WC_);
  Render();
  SCOPED_TRACE("Closer than near");
  VerifyUniformColor(kTerrainColorI, 255u);
  VerifyUniformLabel(RenderLabel::kDontCare);
  VerifyUniformDepth(ImageTraits<PixelType::kDepth32F>::kTooClose);

  // Farther than kZFar.
  X_WC_.set_translation({p_WR(0), p_WR(1), kZFar + 1e-3});
  renderer_->UpdateViewpoint(X_WC_);
  Render();
  SCOPED_TRACE("Farther than far");
  VerifyUniformColor(kTerrainColorI, 255u);
  VerifyUniformLabel(RenderLabel::kDontCare);
  VerifyUniformDepth(ImageTraits<PixelType::kDepth32F>::kTooFar);
}

// Creates a terrain and then positions the camera such that a horizon between
// terrain and sky appears -- no use of "inlier" or "outlier" pixel locations.
TEST_F(RenderEngineVtkTest, HorizonTest) {
  // Camera at the origin, pointing in a direction parallel to the ground.
  RigidTransformd X_WR{RotationMatrixd{AngleAxisd(-M_PI_2, Vector3d::UnitX()) *
                                       AngleAxisd(M_PI_2, Vector3d::UnitY())}};
  Init(X_WR, true);

  const ColorRenderCamera camera(depth_camera_.core(), kShowWindow);
  const auto& intrinsics = camera.core().intrinsics();
  // Returns y in [0, camera.height), index of horizon location in image
  // coordinate system under several assumptions:
  //   - the ground plane is not clipped by kClippingPlaneFar,
  //   - camera is located above the ground (z > 0).
  //   - the ground is a square 100m on a side, centered on world origin.
  //   - camera is located above world origin with a view direction parallel to
  //     the ground (such that the horizon is a horizontal line).
  auto CalcHorizon = [intrinsics](double z) {
    const double kTerrainHalfSize = 50.;
    const double kFocalLength =
        intrinsics.height() * 0.5 / std::tan(0.5 * intrinsics.fov_y());
    // We assume the horizon is *below* the middle of the screen. So, we compute
    // the number of pixels below the screen center the horizon must lie, and
    // then add half screen height to that value to get the number of rows
    // from the top row of the image.
    return 0.5 * intrinsics.height() + z / kTerrainHalfSize * kFocalLength;
  };

  // Verifies v index of horizon at three different camera heights.
  const Vector3d p_WR = X_WR.translation();
  for (const double z : {2., 1., 0.5}) {
    X_WR.set_translation({p_WR(0), p_WR(1), z});
    renderer_->UpdateViewpoint(X_WR);
    ImageRgba8U color(intrinsics.width(), intrinsics.height());
    renderer_->RenderColorImage(camera, &color);

    int actual_horizon{0};
    // This test is looking for the *first* row that isn't exactly sky color.
    // That implies it's starting its search *in the sky*. That implies that the
    // top row is zero and the bottom row is height - 1.
    for (int y = 0; y < intrinsics.height(); ++y) {
      if ((static_cast<uint8_t>(kBgColor.r != color.at(0, y)[0])) ||
          (static_cast<uint8_t>(kBgColor.g != color.at(0, y)[1])) ||
          (static_cast<uint8_t>(kBgColor.b != color.at(0, y)[2]))) {
        actual_horizon = y;
        break;
      }
    }

    const double expected_horizon = CalcHorizon(z);
    ASSERT_NEAR(expected_horizon, actual_horizon, 1.001) << "z = " << z;
  }
}

// TODO(SeanCurtis-TRI): Do texture tests for capsules and ellipsoids as well.

// Performs the shape-centered-in-the-image test with a box.
TEST_F(RenderEngineVtkTest, BoxTest) {
  const auto& intrinsics = depth_camera_.core().intrinsics();
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
      expected_label_ = RenderLabel(1);
      const GeometryId id = GeometryId::get_new_id();

      // For the box, we'll use a special texture that will allow us to detect
      // tiling. The default VTK cube source tiles the texture based on the
      // size of the box. We confirm that doesn't actually happen. We test both
      // the untiled default behavior and the ability to scale the texture.
      PerceptionProperties props = simple_material();
      if (use_texture) {
        props.AddProperty("phong", "diffuse_map",
                          FindResourceOrThrow(
                              "drake/geometry/render/test/diag_gradient.png"));
        props.UpdateProperty("phong", "diffuse", Rgba(1, 1, 1));
        if (texture_scaled) {
          props.AddProperty("phong", "diffuse_scale",
                            Vector2d{texture_scale, texture_scale});
        }
      }
      renderer_->RegisterVisual(id, box, props, RigidTransformd::Identity(),
                                true /* needs update */);
      // We want to position the box so that one corner of the box exactly
      // covers the pixel used for the "inlier test" (w/2, h/2). We can't put
      // the corner at (0, 0, z) (for some depth z) because pixel (w/2, h/2)
      // isn't *centered* on the world origin. We actually want the corner to be
      // half a pixel away from the origin at (px/2, py/2, z), where px and py
      // are the measures of a pixel in meters at the expected depth of the
      // box's leading face.
      // Because we have a radially symmetric lens, px = py and we can compute
      // that measure by with simple trigonometry.
      const double pixel_size = 4 * expected_object_depth_ *
                                tan(intrinsics.fov_y() / 2.0) /
                                intrinsics.height();
      RigidTransformd X_WV{RotationMatrixd{AngleAxisd(M_PI, Vector3d::UnitX())},
                           Vector3d{(-box.width() + pixel_size) * 0.5,
                                    (-box.depth() + pixel_size) * 0.5, 0.625}};
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

        expected_color_ = RgbaColor(ColorI{130, 119, 16}, 255);
        // Quick proof that we're testing for a different color -- we're drawing
        // the red channel from our expected color.
        ASSERT_NE(kTextureColor.r, expected_color_.r);
      } else {
        // Otherwise the expected is simply the texture color of box.png.
        expected_color_ =
            use_texture ? RgbaColor(kTextureColor, 255) : default_color_;
      }

      PerformCenterShapeTest(
          renderer_.get(),
          fmt::format("Box test - {}",
                      use_texture ? (texture_scaled ? "scaled texture"
                                                    : "unscaled texture")
                                  : "diffuse color")
              .c_str());
    }
  }
}

// Performs the shape-centered-in-the-image test with a sphere.
TEST_F(RenderEngineVtkTest, SphereTest) {
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
TEST_F(RenderEngineVtkTest, TransparentSphereTest) {
  RenderEngineVtk renderer;
  InitializeRenderer(X_WC_, true /* add terrain */, &renderer);
  const int int_alpha = 128;
  default_color_ = RgbaColor(kDefaultVisualColor, int_alpha);
  PopulateSphereTest(&renderer);
  const ColorRenderCamera camera(depth_camera_.core(), kShowWindow);
  const auto& intrinsics = camera.core().intrinsics();
  ImageRgba8U color(intrinsics.width(), intrinsics.height());
  renderer.RenderColorImage(camera, &color);

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

  const ScreenCoord inlier = GetInlier(intrinsics);
  EXPECT_TRUE(CompareColor(expect_linear, color, inlier) ||
              CompareColor(expect_quad, color, inlier));
}

// Performs the shape-centered-in-the-image test with a capsule.
TEST_F(RenderEngineVtkTest, CapsuleTest) {
  Init(X_WC_, true);

  // Sets up a capsule.
  const double radius = 0.15;
  const double length = 1.2;
  Capsule capsule(radius, length);
  expected_label_ = RenderLabel(2);
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
TEST_F(RenderEngineVtkTest, CapsuleRotatedTest) {
  Init(X_WC_, true);

  // Sets up a capsule.
  const double radius = 0.15;
  const double length = 1.2;
  Capsule capsule(radius, length);
  expected_label_ = RenderLabel(2);
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
  VerifyOutliers(*renderer_, depth_camera_, name);

  // Verifies the inliers towards the ends of the capsule and ensures its
  // length attribute is respected as opposed to just its radius. This
  // distinguishes it from other shape tests, such as a sphere.
  const ScreenCoord inlier = GetInlier(depth_camera_.core().intrinsics());
  const int offsets[2] = {kHeight / 4, -kHeight / 4};
  const int x = inlier.x;
  for (const int& offset : offsets) {
    const int y = inlier.y + offset;
    const ScreenCoord offset_inlier = {x, y};
    EXPECT_TRUE(CompareColor(expected_color_, color_, offset_inlier))
        << "Color at: " << offset_inlier << " for test: " << name;
    EXPECT_TRUE(IsExpectedDepth(depth_, offset_inlier, expected_object_depth_,
                                kDepthTolerance))
        << "Depth at: " << offset_inlier << " for test: " << name;
    EXPECT_EQ(label_.at(x, y)[0], static_cast<int>(expected_label_))
        << "Label at: " << offset_inlier << " for test: " << name;
  }
}

// Performs the shape-centered-in-the-image test with a cylinder.
TEST_F(RenderEngineVtkTest, CylinderTest) {
  for (const bool use_texture : {false, true}) {
    Init(X_WC_, true);

    // Sets up a cylinder.
    Cylinder cylinder(0.2, 1.2);
    expected_label_ = RenderLabel(2);
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
TEST_F(RenderEngineVtkTest, EllipsoidTest) {
  Init(X_WC_, true);

  // Sets up an ellipsoid.
  const double a = 0.25;
  const double b = 0.4;
  const double c = 0.5;
  Ellipsoid ellipsoid(a, b, c);
  expected_label_ = RenderLabel(2);
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
// be a box). The textured box will be one that is textured via its mtl library.
// We use it to confirm that the render engine is properly _invoking_ the obj
// material handling; the _correctness_ of the material handling is tested in
// render_mesh_test.cc. For the non-textured, we make sure we use a mesh without
// material file or matching foo.png to preclude it being textured.
TEST_F(RenderEngineVtkTest, MeshTest) {
  for (const bool use_texture : {false, true}) {
    Init(X_WC_, true);

    // N.B. box_no_mtl.obj doesn't exist in the source tree and is generated
    // from box.obj by stripping out material data by the build system.
    auto filename =
        use_texture
            ? FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj")
            : FindResourceOrThrow(
                  "drake/geometry/render/test/meshes/box_no_mtl.obj");

    Mesh mesh(filename);
    expected_label_ = RenderLabel(3);
    // We do *not* pass use_texture = true to simple_material(), because we want
    // to see that the texture comes through the natural processing.
    // TODO(SeanCurtis-TRI): The texture applied to box.obj could be applied via
    //  two mechanisms: box.obj.mtl *or* the foo.obj -> foo.png logic. The
    //  assumption is that *this* test doesn't care which mechanism is in play.
    //  If that's wrong, we'll need to create a textured mesh where the
    //  association is strictly unique.
    PerceptionProperties material = simple_material();
    const GeometryId id = GeometryId::get_new_id();
    renderer_->RegisterVisual(id, mesh, material, RigidTransformd::Identity(),
                              true /* needs update */);
    renderer_->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
        {id, RigidTransformd::Identity()}});

    expected_color_ =
        use_texture ? RgbaColor(kTextureColor, 255) : default_color_;
    PerformCenterShapeTest(
        renderer_.get(),
        fmt::format("Mesh test {}", use_texture ? "textured" : "rgba").c_str());
  }
}

// A smaller version of MeshTest. Confirms that VTK supports glTF files as
// Mesh and Convex. Conceptually, the glTF file is a cube with different colors
// on each side. We'll render the cube six times with different orientations to
// confirm that we're seeing what we expect to see. The *structure* of the glTF
// is implicitly testing various aspects of the glTF support including:
//
//  1. Multiple nodes.
//  2. Multiple root nodes.
//  3. Empty nodes (with non-identity transforms).
//  4. Hierarchies.
//  5. Textures.
//     Note: The texture is vertically symmetric -- i.e., you can't tell if the
//     image is upside down or not.
//     TODO(SeanCurtis-TRI): Once the following issue is resolved, replace this
//     with a texture whose vertical orientation matters.
//     https://discourse.vtk.org/t/vtkgltfimporter-loads-textures-upside-down/12113
//  6. Materials.
//  7. Single meshes with multiple materials.
//
// If all of that is processed correctly, we should get a cube with a different
// color on each face. We'll test for those colors.
TEST_F(RenderEngineVtkTest, GltfSupport) {
  struct Face {
    // The expected *illuminated* material color. This color is not exactly
    // the color in the glTF's materials or textures. The PBR shader behaves
    // differently from the phong shader in the other tests. For now, we
    // account for this by putting the observed color in the test. If we change
    // the glTF (or lighting model), we'll need to update these values
    // accordingly.
    Rgba rendered_color;
    RotationMatrixd rotation;
    std::string name;
  };
  Init(X_WC_, true);

  const std::string filename =
      FindResourceOrThrow("drake/geometry/render/test/meshes/rainbow_box.gltf");

  Mesh mesh(filename);
  expected_label_ = RenderLabel(3);
  // Note: Passing diffuse color or texture to a glTF spawns a warning.
  PerceptionProperties material;
  material.AddProperty("label", "id", expected_label_);
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, mesh, material, RigidTransformd::Identity(),
                            true /* needs update */);

  const std::vector<Face> faces{
      {.rendered_color = Rgba(0.078, 0.553, 0.110),
       .rotation = RotationMatrixd(),
       .name = "green"},
      {.rendered_color = Rgba(0.529, 0.259, 0.125),
       .rotation = RotationMatrixd::MakeXRotation(M_PI / 2),
       .name = "orange"},
      {.rendered_color = Rgba(0.553, 0.078, 0.078),
       .rotation = RotationMatrixd::MakeXRotation(M_PI),
       .name = "red"},
      {.rendered_color = Rgba(0.098, 0.078, 0.553),
       .rotation = RotationMatrixd::MakeXRotation(-M_PI / 2),
       .name = "blue"},
      {.rendered_color = Rgba(0.529, 0.529, 0.075),
       .rotation = RotationMatrixd::MakeYRotation(-M_PI / 2),
       .name = "yellow"},
      {.rendered_color = Rgba(0.310, 0.075, 0.529),
       .rotation = RotationMatrixd::MakeYRotation(M_PI / 2),
       .name = "purple"},
  };

  // Render from the original to make sure it's complete and correct.
  for (const auto& face : faces) {
    expected_color_ = face.rendered_color;

    renderer_->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
        {id, RigidTransformd(face.rotation)}});
    PerformCenterShapeTest(
        renderer_.get(),
        fmt::format("glTF test on {} face - original", face.name).c_str());
  }

  // Repeat that from a clone to confirm that the artifacts survived cloning.
  std::unique_ptr<RenderEngine> clone = renderer_->Clone();
  RenderEngineVtk* vtk_clone = dynamic_cast<RenderEngineVtk*>(clone.get());
  for (const auto& face : faces) {
    expected_color_ = face.rendered_color;

    vtk_clone->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
        {id, RigidTransformd(face.rotation)}});
    PerformCenterShapeTest(
        vtk_clone,
        fmt::format("glTF test on {} face - clone", face.name).c_str());
  }
}

// Confirms that meshes/convex referencing a file with an unsupported extension
// are ignored. (There's also an untested one-time warning.)
TEST_F(RenderEngineVtkTest, UnsupportedMeshConvex) {
  Init(X_WC_, false);
  const PerceptionProperties material = simple_material();
  const GeometryId id = GeometryId::get_new_id();

  const Mesh mesh("invalid.fbx");
  EXPECT_FALSE(renderer_->RegisterVisual(id, mesh, material,
                                         RigidTransformd::Identity(),
                                         false /* needs update */));

  const Convex convex("invalid.fbx");
  EXPECT_FALSE(renderer_->RegisterVisual(id, convex, material,
                                         RigidTransformd::Identity(),
                                         false /* needs update */));
}

// Performs the test to cast textures to uchar channels. It depends on the image
// with non-uchar channels being converted to uchar channels losslessly. An
// uint16 image is loaded to prove the existence of the conversion, but this
// test doesn't guarantee universal conversion success.
TEST_F(RenderEngineVtkTest, NonUcharChannelTextures) {
  const ColorRenderCamera camera(depth_camera_.core(), kShowWindow);
  const auto& intrinsics = camera.core().intrinsics();
  const Box box(1.999, 0.55, 0.75);
  expected_label_ = RenderLabel(1);

  // Ensure the texture has non-uchar channels.
  vtkNew<vtkPNGReader> reader;
  const std::string file_path =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box16u.png");
  reader->SetFileName(file_path.c_str());
  reader->Update();
  ASSERT_NE(reader->GetOutput()->GetScalarType(), VTK_UNSIGNED_CHAR);

  // Render a box with an uchar-channel PNG texture.
  ImageRgba8U color_uchar_texture(intrinsics.width(), intrinsics.height());
  {
    RenderEngineVtk renderer;
    InitializeRenderer(X_WC_, false /* no terrain */, &renderer);

    const GeometryId id = GeometryId::get_new_id();
    PerceptionProperties props = simple_material(true);
    renderer.RegisterVisual(id, box, props, RigidTransformd::Identity(), true);
    renderer.RenderColorImage(camera, &color_uchar_texture);
  }

  // Render a box with an uint16-channel PNG texture.
  ImageRgba8U color_uint16_texture(intrinsics.width(), intrinsics.height());
  {
    RenderEngineVtk renderer;
    InitializeRenderer(X_WC_, false /* no terrain */, &renderer);

    const GeometryId id = GeometryId::get_new_id();
    PerceptionProperties props = simple_material(true);
    props.UpdateProperty("phong", "diffuse_map", file_path);
    renderer.RegisterVisual(id, box, props, RigidTransformd::Identity(), true);
    renderer.RenderColorImage(camera, &color_uint16_texture);
  }

  EXPECT_EQ(color_uchar_texture, color_uint16_texture);
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
                                 RenderLabel label)
// Optimizers on some platforms break code and cause test failures. Worse
// still, there is no agreement on attribute spelling.
#ifdef __clang__
      __attribute__((optnone))
#else
      __attribute__((optimize("-O0")))
#endif
  {
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

  const auto& ref_core = depth_camera_.core();
  const std::string& ref_name = ref_core.renderer_name();
  const RigidTransformd ref_X_BS = ref_core.sensor_pose_in_camera_body();
  const render::ClippingRange& ref_clipping = ref_core.clipping();
  const auto& ref_intrinsics = ref_core.intrinsics();
  const int ref_w = ref_intrinsics.width();
  const int ref_h = ref_intrinsics.height();
  const double ref_fov_y = ref_intrinsics.fov_y();

  // Baseline -- confirm that all of the defaults in this test still produce
  // the expected outcome.
  PerformCenterShapeTest(renderer_.get(), "Camera change - baseline",
                         &depth_camera_);

  // Test changes in sensor sizes.
  {
    // Now run it again with a camera with a smaller sensor (quarter area).
    const DepthRenderCamera small_camera{
        {ref_name, {ref_w / 2, ref_h / 2, ref_fov_y}, ref_clipping, ref_X_BS},
        depth_camera_.depth_range()};
    PerformCenterShapeTest(renderer_.get(), "Camera change - small camera",
                           &small_camera);

    // Now run it again with a camera with a bigger sensor (4X area).
    const DepthRenderCamera big_camera{
        {ref_name, {ref_w * 2, ref_h * 2, ref_fov_y}, ref_clipping, ref_X_BS},
        depth_camera_.depth_range()};
    PerformCenterShapeTest(renderer_.get(), "Camera change - big camera",
                           &big_camera);
  }

  // Test changes in fov (larger and smaller).
  {
    const DepthRenderCamera narrow_fov{
        {ref_name, {ref_w, ref_h, ref_fov_y / 2}, ref_clipping, ref_X_BS},
        depth_camera_.depth_range()};
    PerformCenterShapeTest(renderer_.get(), "Camera change - narrow fov",
                           &narrow_fov);

    const DepthRenderCamera wide_fov{
        {ref_name, {ref_w, ref_h, ref_fov_y * 2}, ref_clipping, ref_X_BS},
        depth_camera_.depth_range()};
    PerformCenterShapeTest(renderer_.get(), "Camera change - wide fov",
                           &wide_fov);
  }

  // Test changes to depth range.
  {
    const auto& depth_range = depth_camera_.depth_range();
    const DepthRenderCamera clipping_far_plane{
        depth_camera_.core(),
        {depth_range.min_depth(), expected_outlier_depth_ - 0.1}};
    const float old_outlier_depth = expected_outlier_depth_;
    expected_outlier_depth_ = std::numeric_limits<float>::infinity();
    PerformCenterShapeTest(renderer_.get(),
                           "Camera change - z far clips terrain",
                           &clipping_far_plane);
    // NOTE: Need to restored expected outlier depth for next test.
    expected_outlier_depth_ = old_outlier_depth;

    const DepthRenderCamera clipping_near_plane{
        depth_camera_.core(),
        {expected_object_depth_ + 0.1, depth_range.max_depth()}};
    expected_object_depth_ = 0;
    PerformCenterShapeTest(renderer_.get(), "Camera change - z near clips mesh",
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

  // Case: The engine's default is "don't care".
  {
    ResetExpectations();
    RenderEngineVtk renderer;
    InitializeRenderer(X_WC_, true /* add terrain */, &renderer);

    DRAKE_EXPECT_NO_THROW(populate_default_sphere(&renderer));
    expected_label_ = RenderLabel::kDontCare;
    expected_color_ = RgbaColor(renderer.default_diffuse());

    PerformCenterShapeTest(&renderer, "Default properties; don't care label");
  }

  // Case: Change render engine's default to explicitly be unspecified; must
  // throw.
  {
    RenderEngineVtk renderer{{RenderLabel::kUnspecified, {}}};
    InitializeRenderer(X_WC_, false /* no terrain */, &renderer);

    DRAKE_EXPECT_THROWS_MESSAGE(
        populate_default_sphere(&renderer),
        ".* geometry with the 'unspecified' or 'empty' render labels.*");
  }

  // Case: Change render engine's default to don't care. Label image should
  // report don't care.
  {
    ResetExpectations();
    RenderEngineVtk renderer{{RenderLabel::kDontCare, {}}};
    InitializeRenderer(X_WC_, true /* add terrain */, &renderer);

    DRAKE_EXPECT_NO_THROW(populate_default_sphere(&renderer));
    expected_label_ = RenderLabel::kDontCare;
    expected_color_ = RgbaColor(renderer.default_diffuse());

    PerformCenterShapeTest(&renderer, "Default properties; don't care label");
  }

  // Case: Change render engine's default to invalid default value; must throw.
  {
    for (RenderLabel label :
         {RenderLabel::kEmpty, RenderLabel(1), RenderLabel::kDoNotRender}) {
      DRAKE_EXPECT_THROWS_MESSAGE(
          RenderEngineVtk({label, {}}),
          ".* default render label .* either 'kUnspecified' or 'kDontCare'.*");
    }
  }
}

// This class exists solely for the purpose of injecting an arbitrary texture
// onto an actor and confirm that the texture is preserved over the copy.
// For simplicity, we'll only register shapes that map to vtkActor types.
class TextureSetterEngine : public RenderEngineVtk {
 public:
  TextureSetterEngine() = default;

  // Returns the *first* actor representing the geometry with the given id.
  vtkActor* GetColorActor(GeometryId id) const {
    // 0 is the color index.
    auto* actor = vtkActor::SafeDownCast(props().at(id)[0]);
    DRAKE_DEMAND(actor != nullptr);
    return actor;
  }

  // Reports if the color actor for the geometry with the given `id` has the
  // property texture append by this class's DoRegisterVisual() implementation.
  // This only tests the first actor for the geometry.
  bool GeometryHasColorTexture(GeometryId id,
                               const std::string& texture_name) const {
    vtkActor* actor = GetColorActor(id);
    return actor->GetProperty()->GetTexture(texture_name.c_str()) != nullptr;
  }

  // Applies a texture with the given name to the color actor for the geometry
  // indicated by the given id. This only tests the first actor for the
  // geometry.
  void ApplyColorTextureToGeometry(GeometryId id,
                                   const std::string& texture_name) {
    vtkActor* actor = GetColorActor(id);
    vtkNew<vtkImageData> image_data;
    vtkNew<vtkOpenGLTexture> texture;
    texture->SetRepeat(false);
    texture->InterpolateOn();
    texture->SetInputDataObject(image_data.Get());
    actor->GetProperty()->SetTexture(texture_name.c_str(), texture.Get());
  }

 protected:
  std::unique_ptr<RenderEngine> DoClone() const override {
    return make_unique<TextureSetterEngine>(*this);
  }
};

// Confirms that arbitrary textures assigned to an actors property are preserved
// through cloning.
TEST_F(RenderEngineVtkTest, PreservePropertyTexturesOverClone) {
  const std::string texture_name("test_texture");
  TextureSetterEngine engine;
  Sphere sphere{0.5};
  const GeometryId id = GeometryId::get_new_id();
  const RenderLabel label(12345);  // an arbitrary value.
  PerceptionProperties material;
  material.AddProperty("phong", "diffuse", Vector4d{1.0, 1.0, 1.0, 1.0});
  material.AddProperty("label", "id", label);
  engine.RegisterVisual(id, sphere, material, RigidTransformd(),
                        true /* needs update */);
  engine.ApplyColorTextureToGeometry(id, texture_name);
  ASSERT_TRUE(engine.GeometryHasColorTexture(id, texture_name));
  auto clone_ptr = engine.Clone();
  const TextureSetterEngine* clone =
      dynamic_cast<TextureSetterEngine*>(clone_ptr.get());
  ASSERT_NE(clone, nullptr);
  ASSERT_TRUE(clone->GeometryHasColorTexture(id, texture_name));
}

// Confirm the properties of the fallback camera using the following
// methodology:
//
// Create a scene with a box above a ground plane. The box is parallel to the
// plane. Place the camera in two configurations:
//
//                       A       B
//                       ╱╲    ─┐
//                              │
//                      ┌─┐
//                      │ │                      z
//                      └─┘                    x │
//                                              ╲│
//             ────────────────────────    y ────┘
//
//  A: Box and plane are visible, filling the whole screen (plane behind box);
//     every pixel has the full diffuse color.
//  B: The camera is 45° up from the x-axis, so the normals of the visible faces
//     of box 1 are both 45° away from the camera's view direction. Every pixel
//     will have the same value (√2/2 of the full diffuse value).
//
// These camera angles will allow us to test the following properties:
//
//  1. directional light
//     - All pixels from the same faces share the same normal, therefore the
//       same level of "exposure" (percentage of available light).
//  2. affixed to the camera.
//     - The illumination follows the camera. Exposure dropping from 100% to
//       ~70% from A to B shows this.
//  3. facing in the camera's direction
//     - face normals pointing at the camera will have 100% exposure. Those
//       45° away will have ~70% exposure.
//  4. white light (at normal intensity)
//     - Diffuse color is modulated by the expected light exposure levels.
//  5. no attenuation
//     - the near box and far plane have have the same exposure from view A
//       because it only depends on direction and not distance.
TEST_F(RenderEngineVtkTest, FallbackLight) {
  Vector3d bg_rgb{kBgColor.r / 255.0, kBgColor.g / 255.0, kBgColor.b / 255.0};
  const RenderEngineVtkParams params{.default_clear_color = bg_rgb};
  RenderEngineVtk renderer(params);

  // Load the box.
  const Box box(1, 0.25, 1);
  const render::RenderLabel dummy_label(1);
  PerceptionProperties props;
  const Rgba test_color(0.25, 0.3, 1.0);
  // If there's any doubt that the box is visible, the simplest solution is
  // to change the (phong, diffuse) color for the terrain to something else.
  // The box should then be obviously visible.
  props.AddProperty("phong", "diffuse", test_color);  // match the plane.
  props.AddProperty("label", "id", dummy_label);
  const RigidTransformd X_WB(Vector3d(0, 0, 3));
  const ColorRenderCamera camera(depth_camera_.core(), kShowWindow);
  ImageRgba8U image(camera.core().intrinsics().width(),
                    camera.core().intrinsics().height());
  renderer.RegisterVisual(GeometryId::get_new_id(), box, props, X_WB,
                          false /* needs update */);
  renderer.RegisterVisual(GeometryId::get_new_id(), HalfSpace(), props,
                          RigidTransformd::Identity(),
                          false /* needs update */);

  // The reduced exposure due to the 45-degree angle between all visible face
  // normals and the light direction.
  const double half_sqrt2 = std::sqrt(2.0) / 2;
  const Rgba reduced_color(test_color.r() * half_sqrt2,
                           test_color.g() * half_sqrt2,
                           test_color.b() * half_sqrt2);

  struct Config {
    RigidTransformd X_WR;
    RgbaColor expected_color;
    std::string description;
  };
  const std::vector<Config> configs{
      {.X_WR = RigidTransformd(RotationMatrixd::MakeXRotation(M_PI),
                               X_WB.translation() + Vector3d(0, 0, 1.1)),
       .expected_color = test_color,
       .description = "View A"},
      {.X_WR = RigidTransformd(RotationMatrixd::MakeXRotation(-3 * M_PI / 4),
                               X_WB.translation() + Vector3d(0, -2, 2)),
       .expected_color = reduced_color,
       .description = "View B"}};

  // We want to make sure the lighting configuration survives cloning.
  std::unique_ptr<RenderEngine> clone = renderer.Clone();
  auto* clone_vtk = dynamic_cast<RenderEngineVtk*>(clone.get());
  for (RenderEngineVtk* renderer_ptr : {&renderer, clone_vtk}) {
    for (const auto& config : configs) {
      SCOPED_TRACE(
          fmt::format("{} - {}", config.description,
                      renderer_ptr == clone_vtk ? "Cloned" : "Original"));
      renderer_ptr->UpdateViewpoint(config.X_WR);

      EXPECT_NO_THROW(renderer_ptr->RenderColorImage(camera, &image));

      // We test the images by looking at the colors along a row on the bottom
      // of the image and near the middle of the image. We won't do the top
      // because in view B, the clipped plane reveals the background color.
      //
      // Typically, if one pixel is wrong, many pixels are wrong. So, we use
      // this atypical test spelling to prevent pixel spam for failure. One bad
      // pixel is enough.

      const int mid_height = image.height() / 2;
      for (int r : {0, mid_height}) {
        for (int c = 0; c < image.width(); ++c) {
          RgbaColor dut(image.at(c, r));
          if (!IsColorNear(dut, config.expected_color)) {
            EXPECT_EQ(dut, config.expected_color)
                << "at pixel (" << c << ", " << r << ")";
            break;
          }
        }
      }
    }
  }
}

// This test covers the wiring of the various light parameters. It samples each
// parameter across each light making assertion of what color pixel should be
// found in the center of the image. It confirms that changes to the parameters
// have the expected impact on the color.
//
// This test does *not* test the subtle distinctions between the light types,
// such as the fact that a point light and spotlight have intensity fall off as
// the normal no longer points toward the light. These gross lighting properties
// should be immediately apparent in any rendering.
TEST_F(RenderEngineVtkTest, SingleLight) {
  struct Config {
    LightParameter light;
    RgbaColor expected_color;
    std::string description;
    std::string target_type;
  };

  // 45-degree vertical field of view.
  const ColorRenderCamera camera(depth_camera_.core(), kShowWindow);
  // The camera's position is p_WC = [0, 0, 3]. The ground plane lies on the
  // world's x-y plane. So, the ground is 3.0 meters away from the camera. This
  // will inform attenuation calculations.
  const double dist = 3.0;
  // Camera above the origin, looking down with Wy pointing to the top of the
  // image and Wx to the right.
  const RigidTransformd X_WR(RotationMatrixd::MakeXRotation(M_PI),
                             Vector3d(0, 0, dist));
  ImageRgba8U image(camera.core().intrinsics().width(),
                    camera.core().intrinsics().height());
  const int cx = image.width() / 2;
  const int cy = image.height() / 2;

  const Rgba light_color(0.25, 0.5, 0.75);
  const Rgba kTerrainRgba(kTerrainColorD.r, kTerrainColorD.g, kTerrainColorD.b);
  const Rgba modulated_color = kTerrainRgba * light_color;

  // We'll omit the light type to save space, setting it once in the test loop.

  // The baseline configuration implicitly tests white light, intensity = 1,
  // no attenuation (1, 0, 0), and transformation from camera to world frame of
  // both position and direction of the light.
  const std::vector<Config> configs{
      {.light = {.color = Rgba(1, 1, 1),
                 .attenuation_values = {1, 0, 0},
                 .position = {0, 0, 0},
                 .frame = "camera",
                 .intensity = 1.0,
                 .direction = {0, 0, 1},
                 // If you show the window for spotlight images, the spotlight
                 // circle will exactly fit from image top to bottom.
                 .cone_angle = 22.5},
       .expected_color = kTerrainRgba,
       .description = "Baseline posed in camera"},
      {.light = {.color = Rgba(1, 1, 1),
                 .attenuation_values = {1, 0, 0},
                 .position = {0, 0, dist},
                 .frame = "world",
                 .intensity = 1.0,
                 .direction = {0, 0, -1},
                 .cone_angle = 22.5},
       .expected_color = kTerrainRgba,
       // Should be identical to the baseline image.
       .description = "Baseline posed in world"},
      {.light = {.color = Rgba(1, 1, 1),
                 .attenuation_values = {1, 0, 0},
                 .position = {0, 0, dist},
                 .frame = "camera",
                 .intensity = 1.0,
                 .direction = {0, 0, -1},
                 .cone_angle = 22.5},
       .expected_color = Rgba(0, 0, 0),
       // The lights are positioned badly to illuminate anything.
       .description = "World coordinates in the camera frame - nothing lit!"},
      {.light = {.color = light_color, .cone_angle = 22.5},
       .expected_color = modulated_color,
       .description = "Non-white light color"},
      {.light = {.intensity = 0.1, .cone_angle = 22.5},
       .expected_color = kTerrainRgba.scale_rgb(0.1),
       .description = "Low intensity"},
      {.light = {.intensity = 3.0, .cone_angle = 22.5},
       .expected_color = kTerrainRgba.scale_rgb(3),
       .description = "High intensity"},
      {.light = {.attenuation_values = {2, 0, 0}, .cone_angle = 22.5},
       .expected_color = kTerrainRgba.scale_rgb(0.5),
       .description = "Non-unit constant attenuation"},
      {.light = {.attenuation_values = {0, 1, 0}, .cone_angle = 22.5},
       .expected_color = kTerrainRgba.scale_rgb(1 / dist),
       .description = "Linear attenuation"},
      {.light = {.attenuation_values = {0, 0, 1}, .cone_angle = 22.5},
       .expected_color = kTerrainRgba.scale_rgb(1 / (dist * dist)),
       .description = "Quadratic attenuation"},
      {.light = {.cone_angle = 0},
       .expected_color = Rgba(0, 0, 0),
       .description = "Zero cone angle",
       .target_type = "spot"}};

  for (const auto& config : configs) {
    for (const auto& l_type : {"point", "spot", "directional"}) {
      if (!config.target_type.empty() && l_type != config.target_type) {
        continue;
      }
      // In VTK, the attenuation values don't affect directional lights.
      if (l_type == std::string("directional") &&
          config.description.find("attenuation") != std::string::npos) {
        continue;
      }
      SCOPED_TRACE(
          fmt::format("{} - {}", fmt_streamed(l_type), config.description));
      LightParameter test_light = config.light;
      test_light.type = l_type;
      const RenderEngineVtkParams params{.lights = {test_light}};
      RenderEngineVtk renderer(params);

      InitializeRenderer(X_WR, true /* add terrain */, &renderer);

      // We want to make sure the lighting configuration survives cloning.
      std::unique_ptr<RenderEngine> clone = renderer.Clone();
      auto* clone_vtk = dynamic_cast<RenderEngineVtk*>(clone.get());
      for (const RenderEngineVtk* renderer_ptr : {&renderer, clone_vtk}) {
        SCOPED_TRACE(renderer_ptr == clone_vtk ? "Cloned" : "Original");
        EXPECT_NO_THROW(renderer_ptr->RenderColorImage(camera, &image));

        const RgbaColor test_color(image.at(cx, cy));
        EXPECT_TRUE(IsColorNear(test_color, config.expected_color))
            << "  test color: " << test_color << "\n"
            << "  expected color: " << config.expected_color;
      }
    }
  }
}

// Quick test to make sure that lights combine. We'll intentionally use more
// lights than RenderEngineGl allows for to confirm that RenderEngineVtk doesn't
// share the limit.
TEST_F(RenderEngineVtkTest, MultiLights) {
  const ColorRenderCamera camera(depth_camera_.core(), kShowWindow);
  const RigidTransformd X_WR(RotationMatrixd::MakeXRotation(M_PI),
                             Vector3d(0, 0, 3));
  ImageRgba8U image(camera.core().intrinsics().width(),
                    camera.core().intrinsics().height());
  const int cx = image.width() / 2;
  const int cy = image.height() / 2;

  // We have three conceptual lights. The *conceptual* lights are pointing
  // directly at the image center, but their total intensity is 0.75. So, we
  // should get 75% of the diffuse color. To test the non-limits on the number
  // of lights, we'll duplicate each light with half the intensity.
  const RenderEngineVtkParams params{
      .lights = {{.type = "point", .intensity = 0.25 * 0.5},
                 {.type = "point", .intensity = 0.25 * 0.5},
                 {.type = "spot", .intensity = 0.25 * 0.5, .cone_angle = 45},
                 {.type = "spot", .intensity = 0.25 * 0.5, .cone_angle = 45},
                 {.type = "directional", .intensity = 0.25 * 0.5},
                 {.type = "directional", .intensity = 0.25 * 0.5}}};
  RenderEngineVtk renderer(params);

  InitializeRenderer(X_WR, true /* add terrain */, &renderer);

  EXPECT_NO_THROW(renderer.RenderColorImage(camera, &image));

  const RgbaColor test_color(image.at(cx, cy));
  const Rgba terrain_rgba(kTerrainColorD.r, kTerrainColorD.g, kTerrainColorD.b);
  const RgbaColor expected_color = terrain_rgba.scale_rgb(0.75);
  EXPECT_TRUE(IsColorNear(test_color, expected_color))
      << "  test color: " << test_color << "\n"
      << "  expected color: " << expected_color;
}

namespace {

// Defines the relationship between two adjacent pixels in a rendering of a box.
// Used to help find the edges of a rendered box.
enum AdjacentPixel { Same, GroundToBox, BoxToGround };

// Note: These overloads work because the pixel "types" ::T for each image type
// is distinct.

AdjacentPixel Compare(const typename ImageDepth32F::T* curr_pixel,
                      const typename ImageDepth32F::T* next_pixel) {
  // Box depth < ground depth.
  if (*curr_pixel < *next_pixel) return BoxToGround;
  if (*curr_pixel > *next_pixel) return GroundToBox;
  return Same;
}

AdjacentPixel Compare(const typename ImageRgba8U::T* curr_pixel,
                      const typename ImageRgba8U::T* next_pixel) {
  const RgbaColor ground(kTerrainColorI, 255);
  const RgbaColor curr(curr_pixel);
  const RgbaColor next(next_pixel);

  const bool curr_is_ground =
      curr.r == ground.r && curr.g == ground.g && curr.b == ground.b;
  const bool next_is_ground =
      next.r == ground.r && next.g == ground.g && next.b == ground.b;
  if (!curr_is_ground && next_is_ground) return BoxToGround;
  if (curr_is_ground && !next_is_ground) return GroundToBox;
  return Same;
}

AdjacentPixel Compare(const typename ImageLabel16I::T* curr_pixel,
                      const typename ImageLabel16I::T* next_pixel) {
  const bool curr_is_box = *curr_pixel == kDefaultLabel;
  const bool next_is_box = *next_pixel == kDefaultLabel;

  if (curr_is_box && !next_is_box) return BoxToGround;
  if (!curr_is_box && next_is_box) return GroundToBox;
  return Same;
}

// Find the edges of the box in the image. Using the single-channel depth
// image simplifies finding the edges. The edges are encoded as follows:
// left, top, right, bottom. (such that right > left, and top > bottom). It
// represents the row/column in the image that represents the left-most,
// top-most, etc. extents of the box. It is assumed that the box edges run
// parallel with the image rows and columns and the box is wholly contained
// within the image.
template <typename ImageType>
Vector4<int> FindBoxEdges(const ImageType& image) {
  using T = typename ImageType::T;
  Vector4<int> edges{-1, -1, -1, -1};

  for (int x = 0; x < image.width() - 1; ++x) {
    for (int y = 0; y < image.height() - 1; ++y) {
      const T* curr_pixel = image.at(x, y);

      // Look for edge between current pixel and pixel below.
      const T* bottom_pixel = image.at(x, y + 1);
      const AdjacentPixel bottom_result = Compare(curr_pixel, bottom_pixel);
      if (bottom_result == GroundToBox) {
        // Current lies on the ground, next lies on the box; bottom edge.
        DRAKE_DEMAND(edges(3) == -1 || edges(3) == y + 1);
        edges(3) = y + 1;
      } else if (bottom_result == BoxToGround) {
        // Current lies on the box, next lies on the ground; top edges.
        DRAKE_DEMAND(edges(1) == -1 || edges(1) == y);
        edges(1) = y;
      }

      // Look for edge between current pixel and pixel to the right.
      const T* right_pixel = image.at(x + 1, y);
      const AdjacentPixel right_result = Compare(curr_pixel, right_pixel);
      if (right_result == GroundToBox) {
        // Current lies on the ground, next lies on the box; left edge.
        DRAKE_DEMAND(edges(0) == -1 || edges(0) == x + 1);
        edges(0) = x + 1;
      } else if (right_result == BoxToGround) {
        // Current lies on the box, next lies on the ground; right edge.
        DRAKE_DEMAND(edges(2) == -1 || edges(2) == x);
        edges(2) = x;
      }
    }
  }

  return edges;
}

}  // namespace

// Confirm that all of the intrinsics values have the expected contribution.
// We'll render a cube multiple times. Each rendering will have its own camera
// configuration and we'll predict the contents of the image relative to a
// baseline image, based on the change in rendering properties.
TEST_F(RenderEngineVtkTest, IntrinsicsAndRenderProperties) {
  // TODO(11965) Now that the creation of the projection matrix is part
  //  of RenderCameraCore (and tested there), this could be simplified. We rely
  //  on the *correctness* of the projection matrix and merely confirm that
  //  it is being computed at all. That would eliminate the many tweaks and
  //  comparisons. Consider simplifying this when the render engine test
  //  infrastructure is refactored.
  Init(X_WC_, true /* add_terrain */);
  PopulateSimpleBoxTest(renderer_.get());

  // Reference case: Vanilla, ideal camera: isotropic, centered lens. We'll
  // define future expectations relative to this one. The actual parameters are
  // arbitrary. The goal is to place the box inside the image. The focal
  // length should not be too large, otherwise the box edges cannot be
  // identified in the image.
  const int w = 300;
  const int h = w;
  const double fx = 270;
  const double fy = fx;
  const double cx = w / 2.0 + 0.5;
  const double cy = h / 2.0 + 0.5;
  const double min_depth = 0.1;
  const double max_depth = 5;
  const double clip_n = 0.01;
  const double clip_f = 10.0;

  const CameraInfo ref_intrinsics{w, h, fx, fy, cx, cy};
  const ColorRenderCamera ref_color_camera{
      {"n/a", ref_intrinsics, {clip_n, clip_f}, {}}, kShowWindow};
  const DepthRenderCamera ref_depth_camera{
      {"n/a", ref_intrinsics, {clip_n, clip_f}, {}}, {min_depth, max_depth}};

  ImageRgba8U ref_color(ref_intrinsics.width(), ref_intrinsics.height());
  ImageDepth32F ref_depth(ref_intrinsics.width(), ref_intrinsics.height());
  ImageLabel16I ref_label(ref_intrinsics.width(), ref_intrinsics.height());
  renderer_->RenderColorImage(ref_color_camera, &ref_color);
  renderer_->RenderDepthImage(ref_depth_camera, &ref_depth);
  renderer_->RenderLabelImage(ref_color_camera, &ref_label);

  // Confirm all edges were found, the box is square and centered in the
  // image.
  const Vector4<int> ref_box_edges = FindBoxEdges(ref_depth);
  ASSERT_TRUE((ref_box_edges.array() > -1).all())
      << fmt::to_string(fmt_eigen(ref_box_edges.transpose()));
  const int ref_box_width = ref_box_edges(2) - ref_box_edges(0);
  const int ref_box_height = ref_box_edges(1) - ref_box_edges(3);
  ASSERT_EQ(ref_box_width, ref_box_height);
  ASSERT_EQ((ref_box_edges(2) + ref_box_edges(0)) / 2, w / 2);
  ASSERT_EQ((ref_box_edges(1) + ref_box_edges(3)) / 2, h / 2);

  {
    // Also confirm the box is positioned the same in color and label images.
    const Vector4<int> color_edges = FindBoxEdges(ref_color);
    ASSERT_EQ(color_edges, ref_box_edges)
        << fmt::to_string(fmt_eigen(color_edges));
    const Vector4<int> label_edges = FindBoxEdges(ref_label);
    ASSERT_EQ(label_edges, ref_box_edges)
        << fmt::to_string(fmt_eigen(label_edges));
  }

  {
    // Case: Modify image dimensions, focal length, and principal point; the
    // box should move and stretch.
    const int w2 = w + 100;
    const int h2 = h + 50;
    // There are limits on how much *bigger* a focal length can be. Too big
    // and the box will no longer fit in the image and the test will fail
    // because FindBoxEdges()'s assumptions will be broken.
    const double fx2 = fx * 1.1;
    const double fy2 = fy * 0.75;
    const double offset_x = 10;
    const double offset_y = -15;
    const double cx2 = w2 / 2.0 + 0.5 + offset_x;
    const double cy2 = h2 / 2.0 + 0.5 + offset_y;
    const CameraInfo intrinsics{w2, h2, fx2, fy2, cx2, cy2};
    const ColorRenderCamera color_camera{
        {"n/a", intrinsics, {clip_n, clip_f}, {}}, kShowWindow};
    const DepthRenderCamera depth_camera{
        {"n/a", intrinsics, {clip_n, clip_f}, {}}, {min_depth, max_depth}};

    // We don't test for image/camera size mismatches here. That has been
    // tested in render_engine_test.cc.
    ImageRgba8U color(intrinsics.width(), intrinsics.height());
    ImageDepth32F depth(intrinsics.width(), intrinsics.height());
    ImageLabel16I label(intrinsics.width(), intrinsics.height());
    renderer_->RenderColorImage(color_camera, &color);
    renderer_->RenderDepthImage(depth_camera, &depth);
    renderer_->RenderLabelImage(color_camera, &label);

    // We expect the following effects based on the change in intrinsics.
    //
    //  - The center of the box moves <offset_x, offset_y> pixels from the
    //    center of the new image.
    //  - Changing the focal length will change the measure of the box in the
    //    corresponding dimension. So, if ratio_x = fx2 / fx, then
    //    width2 = width * ratio_x. (Similarly for height.)
    //  - Changing the image size alone will *not* change the appearance of the
    //    box (unless the image size is reduced sufficiently to truncate the
    //    box). With a fixed focal length, changing image size will merely
    //    increase or decrease the amount of image around the box.

    const Vector4<int> test_edges = FindBoxEdges(depth);
    ASSERT_TRUE((test_edges.array() > -1).all())
        << fmt::to_string(fmt_eigen(test_edges.transpose()));
    const int test_box_width = test_edges(2) - test_edges(0);
    const int test_box_height = test_edges(1) - test_edges(3);

    // Confirm the box gets squished.
    const double fx_ratio = fx2 / fx;
    const double fy_ratio = fy2 / fy;
    EXPECT_NEAR(test_box_width, ref_box_width * fx_ratio, 1);
    EXPECT_NEAR(test_box_height, ref_box_height * fy_ratio, 1);

    // Confirm that its center is translated.
    EXPECT_NEAR((test_edges(0) + test_edges(2)) / 2.0, w2 / 2.0 + offset_x, 1.0)
        << fmt::to_string(fmt_eigen(test_edges.transpose()));
    EXPECT_NEAR((test_edges(1) + test_edges(3)) / 2.0, h2 / 2.0 + offset_y, 1.0)
        << fmt::to_string(fmt_eigen(test_edges.transpose()));

    {
      // Also confirm it matches for color and label.
      const Vector4<int> color_edges = FindBoxEdges(color);
      ASSERT_EQ(color_edges, test_edges)
          << fmt::to_string(fmt_eigen(color_edges.transpose()));
      const Vector4<int> label_edges = FindBoxEdges(label);
      ASSERT_EQ(label_edges, test_edges)
          << fmt::to_string(fmt_eigen(label_edges.transpose()));
    }
  }

  {
    // Case: far plane is in front of all geometry; nothing visible.
    const double n_alt = expected_object_depth_ * 0.1;
    const double f_alt = expected_object_depth_ * 0.9;
    const ColorRenderCamera color_camera{
        {"n/a", ref_intrinsics, {n_alt, f_alt}, {}}, kShowWindow};
    // Set depth range to clipping range so we don't take a chance with the
    // depth range lying outside the clipping range.
    const DepthRenderCamera depth_camera{
        {"n/a", ref_intrinsics, {n_alt, f_alt}, {}}, {n_alt, f_alt}};
    ImageRgba8U color(ref_intrinsics.width(), ref_intrinsics.height());
    ImageDepth32F depth(ref_intrinsics.width(), ref_intrinsics.height());
    ImageLabel16I label(ref_intrinsics.width(), ref_intrinsics.height());
    renderer_->RenderColorImage(color_camera, &color);
    renderer_->RenderDepthImage(depth_camera, &depth);
    renderer_->RenderLabelImage(color_camera, &label);

    SCOPED_TRACE("Far plane in front of scene");
    VerifyUniformColor(kBgColor, 255u, &color);
    VerifyUniformLabel(RenderLabel::kEmpty, &label);
    VerifyUniformDepth(std::numeric_limits<float>::infinity(), &depth);
  }

  {
    // Case: near plane too far; nothing visible.
    // We're assuming the box has an edge length <= 2.
    const double n_alt = expected_object_depth_ + 2.1;
    const double f_alt = expected_object_depth_ + 4.1;
    const ColorRenderCamera color_camera{
        {"n/a", ref_intrinsics, {n_alt, f_alt}, {}}, kShowWindow};
    // Set depth range to clipping range so we don't take a chance with the
    // depth range lying outside the clipping range.
    const DepthRenderCamera depth_camera{
        {"n/a", ref_intrinsics, {n_alt, f_alt}, {}}, {n_alt, f_alt}};
    ImageRgba8U color(ref_intrinsics.width(), ref_intrinsics.height());
    ImageDepth32F depth(ref_intrinsics.width(), ref_intrinsics.height());
    ImageLabel16I label(ref_intrinsics.width(), ref_intrinsics.height());
    renderer_->RenderColorImage(color_camera, &color);
    renderer_->RenderDepthImage(depth_camera, &depth);
    renderer_->RenderLabelImage(color_camera, &label);

    SCOPED_TRACE("Near plane beyond scene");
    VerifyUniformColor(kBgColor, 255u, &color);
    VerifyUniformLabel(RenderLabel::kEmpty, &label);
    VerifyUniformDepth(std::numeric_limits<float>::infinity(), &depth);
  }

  {
    // Case: configure the depth range to lie _between_ the depth of the box's
    // leading face and the ground. The box face should register as too close
    // and the ground should be too far.
    const double min_alt = expected_object_depth_ + 0.1;
    const double max_alt = expected_outlier_depth_ - 0.1;
    const DepthRenderCamera depth_camera{
        {"n/a", ref_intrinsics, {clip_n, clip_f}, {}}, {min_alt, max_alt}};
    ImageDepth32F depth(ref_intrinsics.width(), ref_intrinsics.height());
    renderer_->RenderDepthImage(depth_camera, &depth);

    // Confirm pixel in corner (ground) and pixel in center (box).
    EXPECT_TRUE(IsExpectedDepth(depth, ScreenCoord{w / 2, h / 2},
                                ImageTraits<PixelType::kDepth32F>::kTooClose,
                                0.0));
    EXPECT_TRUE(IsExpectedDepth(depth, ScreenCoord{0, 0},
                                ImageTraits<PixelType::kDepth32F>::kTooFar,
                                0.0));
  }

  {
    // Case: The whole sensor depth range lies *in front* of the box's leading
    // face. The whole depth image should report as too far.
    const double min_alt = expected_object_depth_ * 0.5;
    const double max_alt = expected_object_depth_ * 0.9;
    const DepthRenderCamera depth_camera{
        {"n/a", ref_intrinsics, {clip_n, clip_f}, {}}, {min_alt, max_alt}};
    ImageDepth32F depth(ref_intrinsics.width(), ref_intrinsics.height());
    renderer_->RenderDepthImage(depth_camera, &depth);

    // Confirm pixel in corner (ground) and pixel in center (box).
    EXPECT_TRUE(IsExpectedDepth(depth, ScreenCoord{w / 2, h / 2},
                                ImageTraits<PixelType::kDepth32F>::kTooFar,
                                0.0));
    EXPECT_TRUE(IsExpectedDepth(depth, ScreenCoord{0, 0},
                                ImageTraits<PixelType::kDepth32F>::kTooFar,
                                0.0));
  }

  {
    // Case: The whole sensor depth range lies beyond the ground. The whole
    // depth image should report as too close. This result is in stark contrast
    // to *clipping* the geometry away -- there, the distance reports as too
    // far.
    // We're assuming the box has an edge length <= 2.
    const double min_alt = expected_object_depth_ + 2.1;
    const double max_alt = expected_object_depth_ + 4.1;
    const DepthRenderCamera depth_camera{
        {"n/a", ref_intrinsics, {clip_n, clip_f}, {}}, {min_alt, max_alt}};
    ImageDepth32F depth(ref_intrinsics.width(), ref_intrinsics.height());
    renderer_->RenderDepthImage(depth_camera, &depth);

    // Confirm pixel in corner (ground) and pixel in center (box).
    EXPECT_TRUE(IsExpectedDepth(depth, ScreenCoord{w / 2, h / 2},
                                ImageTraits<PixelType::kDepth32F>::kTooClose,
                                0.0));
    EXPECT_TRUE(IsExpectedDepth(depth, ScreenCoord{0, 0},
                                ImageTraits<PixelType::kDepth32F>::kTooClose,
                                0.0));
  }
}

}  // namespace
}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
