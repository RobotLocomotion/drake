#include "drake/geometry/render/gl_renderer/render_engine_gl.h"

#include <array>
#include <cstring>
#include <unordered_map>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/render_label.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/sensors/color_palette.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render {

// Friend class that gives the tests access to a RenderEngineGl's OpenGlContext.
class RenderEngineGlTester {
 public:
  /* Constructs a tester on the given engine. The tester keeps a reference to
   the given `engine`; the engine must stay alive at least as long as the
   tester.  */
  explicit RenderEngineGlTester(const RenderEngineGl* engine)
      : engine_(*engine) {
    DRAKE_DEMAND(engine != nullptr);
  }

  const internal::OpenGlContext& opengl_context() const {
    return *engine_.opengl_context_;
  }

  const internal::OpenGlGeometry GetMesh(const std::string& filename) const {
    return const_cast<RenderEngineGl&>(engine_).GetMesh(filename);
  }

 private:
  const RenderEngineGl& engine_;
};

namespace {

using Eigen::AngleAxisd;
using Eigen::Translation3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransformd;
using math::RotationMatrixd;
using std::make_unique;
using std::unique_ptr;
using std::unordered_map;
using systems::sensors::CameraInfo;
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
const double kClipNear = 0.01;
const double kClipFar = 100.0;
const double kZNear = 0.1;
const double kZFar = 5.;
const double kFovY = M_PI_4;
const bool kShowWindow = false;

// Each channel of the color image must be within the expected color +/- 1
// (where each channel is in the range [0, 255]).
const double kColorPixelTolerance = 1;

// NOTE: The depth tolerance is this large mostly due to the combination of
// several factors:
//   - the sphere test (sphere against terrain)
//   - The even-valued window dimensions
//   - the tests against various camera image sizes
// The explanation is as follows. The distance to the sphere is only exactly
// 2 at the point of the sphere directly underneath the camera (the sphere's
// "peak"). However, with an even-valued window dimension, we never really
// sample that point. We sample the center of pixels all evenly arrayed around
// that point. So, that introduces some error. As the image gets *smaller* the
// pixels get bigger and so the distance away from the peak center increases,
// which, in turn, increase the measured distance for the fragment. This
// tolerance accounts for the test case where one image has pixels that are *4X*
// larger (in area) than the default image size.
const double kDepthTolerance = 1e-3;  // meters.

// Background (sky) and terrain colors.
const Rgba kBgColor{254 / 255.0, 127 / 255.0, 0.0, 1.0};
const Rgba kTerrainColor{0, 0, 0, 1};
// box.png contains a single pixel with the color (4, 241, 33). If the image
// changes, the expected color would likewise have to change.
const Rgba kTextureColor{4 / 255.0, 241 / 255.0, 33 / 255.0, 1.0};

// Provide a default visual color for this tests -- it is intended to be
// different from the default color of the OpenGL render engine.
const Rgba kDefaultVisualColor{229 / 255.0, 229 / 255.0, 229/ 255.0, 1.0};
const float kDefaultDistance{3.f};

const RenderLabel kDefaultLabel{13531};

// Values to be used with the "centered shape" tests.
// The amount inset from the edge of the images to *still* expect terrain
// values.
static constexpr int kInset{10};

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

// Utility struct for doing color testing; provides three mechanisms for
// creating a common rgba color. We get colors from images (as a pointer to
// unsigned bytes, as a (ColorI, alpha) pair, and from a normalized color. It's
// nice to articulate tests without having to worry about those details.
struct RgbaColor {
  RgbaColor(const ColorI& c, int alpha)
      : r(c.r), g(c.g), b(c.b), a(alpha) {}
  explicit RgbaColor(const uint8_t* p) : r(p[0]), g(p[1]), b(p[2]), a(p[3]) {}
  // We'll allow *implicit* conversion from Rgba to RgbaColor to increase the
  // utility of IsColorNear(), but only in the scope of this test.
  // NOLINTNEXTLINE(runtime/explicit)
  RgbaColor(const Rgba& rgba)
      : r(static_cast<int>(rgba.r() * 255)),
        g(static_cast<int>(rgba.g() * 255)),
        b(static_cast<int>(rgba.b() * 255)),
        a(static_cast<int>(rgba.a() * 255)) {}
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
  return (abs(expected.r - tested.r) <= tolerance &&
      abs(expected.g - tested.g) <= tolerance &&
      abs(expected.b - tested.b) <= tolerance &&
      abs(expected.a - tested.a) <= tolerance);
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
         << "At pixel " << p << "\n  Expected: " << expected
         << "\n  Found: " << tested << "\n  with tolerance: " << tolerance;
}

class RenderEngineGlTest : public ::testing::Test {
 public:
  RenderEngineGlTest()
      : color_(kWidth, kHeight),
        depth_(kWidth, kHeight),
        label_(kWidth, kHeight),
        // Looking straight down from 3m above the ground.
        X_WR_(RotationMatrixd{AngleAxisd(M_PI, Vector3d::UnitY()) *
                              AngleAxisd(-M_PI_2, Vector3d::UnitZ())},
              {0, 0, kDefaultDistance}),
        geometry_id_(GeometryId::get_new_id()) {}

 protected:
  // Method to allow the normal case (render with the built-in renderer against
  // the default camera) to the member images with default window visibility.
  // This interface allows that to be completely reconfigured by the calling
  // test.
  void Render(RenderEngineGl* renderer = nullptr,
              const DepthRenderCamera* camera_in = nullptr,
              ImageRgba8U* color_out = nullptr,
              ImageDepth32F* depth_in = nullptr,
              ImageLabel16I* label_in = nullptr) {
    if (!renderer) renderer = renderer_.get();
    const DepthRenderCamera& depth_camera =
        camera_in ? *camera_in : depth_camera_;
    const ColorRenderCamera color_camera(depth_camera.core(), kShowWindow);
    ImageLabel16I* label = label_in ? label_in : &label_;
    ImageDepth32F* depth = depth_in ? depth_in : &depth_;
    ImageRgba8U* color = color_out ? color_out : &color_;
    EXPECT_NO_THROW(renderer->RenderDepthImage(depth_camera, depth));
    EXPECT_NO_THROW(renderer->RenderLabelImage(color_camera, label));
    EXPECT_NO_THROW(renderer->RenderColorImage(color_camera, color));
  }

  // Confirms that all pixels in the member color image have the same value.
  void VerifyUniformColor(const Rgba& rgba,
                          const ImageRgba8U* color = nullptr) {
    if (color == nullptr) color = &color_;
    const RgbaColor test_color{rgba};
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

  // Report the coordinates of the set of "outlier" pixels for a given set of
  // camera properties. These are the pixels that are drawn on by the
  // background (possibly flat plane, possibly sky) and not the primary
  // geometry.
  static std::vector<ScreenCoord> GetOutliers(const CameraInfo& intrinsics) {
    return std::vector<ScreenCoord>{
        {kInset, kInset},
        {kInset, intrinsics.height() - kInset - 1},
        {intrinsics.width() - kInset - 1, intrinsics.height() - kInset - 1},
        {intrinsics.width() - kInset - 1, kInset}};
  }

  // Compute the coordinate of the pixel that is drawn on by the primary
  // geometry given a set of camera properties.
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
               << "Expected depth at " << coord << " to be "
               << "infinity. Found: " << actual_depth;
      }
    } else {
      const float delta = std::abs(expected_depth - actual_depth);
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

  // Verifies the "outlier" pixels for the given camera belong to the terrain.
  // If images are provided, the given images will be tested, otherwise the
  // member images will be tested.
  void VerifyOutliers(const RenderEngineGl& renderer,
                      const DepthRenderCamera& camera,
                      const ImageRgba8U* color_in = nullptr,
                      const ImageDepth32F* depth_in = nullptr,
                      const ImageLabel16I* label_in = nullptr) const {
    const ImageRgba8U& color = color_in ? *color_in : color_;
    const ImageDepth32F& depth = depth_in ? *depth_in : depth_;
    const ImageLabel16I& label = label_in ? *label_in : label_;

    for (const auto& screen_coord : GetOutliers(camera.core().intrinsics())) {
      EXPECT_TRUE(CompareColor(expected_outlier_color_, color, screen_coord));
      EXPECT_TRUE(IsExpectedDepth(depth, screen_coord, expected_outlier_depth_,
                                  kDepthTolerance))
          << "Depth at: " << screen_coord;
      EXPECT_EQ(label.at(screen_coord.x, screen_coord.y)[0],
                expected_outlier_label_)
          << "Label at: " << screen_coord;
    }
  }

  void SetUp() override {
    ResetExpectations();
  }

  // Tests that don't instantiate their own renderers should invoke this.
  void Init(const RigidTransformd& X_WR, bool add_terrain = false) {
    // TODO(SeanCurtis-TRI): When GCC supports non-trivial designated
    //  initialization turn this back into:
    //  const RenderEngineGlParams params{.default_clear_color = kBgColor};
    RenderEngineGlParams params;
    params.default_clear_color = kBgColor;
    renderer_ = make_unique<RenderEngineGl>(params);
    InitializeRenderer(X_WR, add_terrain, renderer_.get());
    // Ensure that we the test default visual color is different from the
    // render engine's default color.
    EXPECT_FALSE(IsColorNear(kDefaultVisualColor,
                             renderer_->parameters().default_diffuse));
  }

  // Tests that instantiate their own renderers can initialize their renderers
  // with this method.
  void InitializeRenderer(const RigidTransformd& X_WR, bool add_terrain,
                          RenderEngineGl* engine) {
    engine->UpdateViewpoint(X_WR);

    if (add_terrain) {
      PerceptionProperties material;
      material.AddProperty("label", "id", RenderLabel::kDontCare);
      material.AddProperty("phong", "diffuse", kTerrainColor);
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
    material.AddProperty("phong", "diffuse", default_color_);
    material.AddProperty("label", "id", expected_label_);
    if (use_texture) {
      material.AddProperty(
          "phong", "diffuse_map",
          FindResourceOrThrow(
              "drake/geometry/render/test/meshes/box.png"));
    }
    return material;
  }

  // Resets all expected values to the initial, default values.
  void ResetExpectations() {
    expected_color_ = RgbaColor{kDefaultVisualColor};
    expected_outlier_color_ = RgbaColor{kTerrainColor};
    expected_outlier_depth_ = 3.0f;
    expected_object_depth_ = 2.0f;
    // We expect each test to explicitly set this.
    expected_label_ = RenderLabel();
    expected_outlier_label_ = RenderLabel::kDontCare;
  }

  // Populates the given renderer with the sphere required for
  // PerformCenterShapeTest().
  void PopulateSphereTest(RenderEngineGl* renderer, bool use_texture = false) {
    const double r = 0.5;
    Sphere sphere{r};
    expected_label_ = RenderLabel(12345);  // an arbitrary value.
    renderer->RegisterVisual(geometry_id_, sphere, simple_material(use_texture),
                             RigidTransformd::Identity(),
                             true /* needs update */);
    RigidTransformd X_WV{Vector3d{0, 0, r}};
    X_WV_.clear();
    X_WV_.insert({geometry_id_, X_WV});
    renderer->UpdatePoses(X_WV_);
  }

  void PopulateSimpleBoxTest(RenderEngineGl* renderer) {
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
        X_WR_.translation()(2) - length / 2 - expected_object_depth_;
    RigidTransformd X_WV{Vector3d{0, 0, p_WVo_z}};
    renderer->UpdatePoses(
        unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});
    expected_color_ = RgbaColor{default_color_};
  }

  // Performs the work to test the rendering with a sphere centered in the
  // image. To pass, the renderer will have to have been populated with a
  // compliant sphere and camera configuration (e.g., PopulateSphereTest()).
  void PerformCenterShapeTest(RenderEngineGl* renderer,
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

    VerifyCenterShapeTest(*renderer, cam, color, depth, label);
  }

  void VerifyCenterShapeTest(const RenderEngineGl& renderer,
                             const DepthRenderCamera& camera,
                             const ImageRgba8U& color,
                             const ImageDepth32F& depth,
                             const ImageLabel16I& label) const {
    VerifyOutliers(renderer, camera, &color, &depth, &label);

    // Verifies inside the sphere.
    const ScreenCoord inlier = GetInlier(camera.core().intrinsics());

    EXPECT_TRUE(CompareColor(expected_color_, color, inlier))
              << "Color at: " << inlier;
    EXPECT_TRUE(
        IsExpectedDepth(depth, inlier, expected_object_depth_, kDepthTolerance))
        << "Depth at: " << inlier;
    EXPECT_EQ(label.at(inlier.x, inlier.y)[0],
              static_cast<int>(expected_label_))
        << "Label at: " << inlier;
  }

  RgbaColor expected_color_{kDefaultVisualColor};
  RgbaColor expected_outlier_color_{kTerrainColor};
  float expected_outlier_depth_{kDefaultDistance};
  float expected_object_depth_{2.f};
  RenderLabel expected_label_;
  RenderLabel expected_outlier_label_{RenderLabel::kDontCare};
  Rgba default_color_{kDefaultVisualColor};

  // We store a reference depth camera; we can always derive a color camera
  // from it; they have the same intrinsics and we grab the global kShowWindow.
  const DepthRenderCamera depth_camera_{
      {"unused", {kWidth, kHeight, kFovY}, {kClipNear, kClipFar}, {}},
      {kZNear, kZFar}};

  ImageRgba8U color_;
  ImageDepth32F depth_;
  ImageLabel16I label_;
  RigidTransformd X_WR_;
  GeometryId geometry_id_;

  // The pose of the sphere created in PopulateSphereTest().
  unordered_map<GeometryId, RigidTransformd> X_WV_;

  unique_ptr<RenderEngineGl> renderer_;
};

// Tests an empty image -- confirms that it clears to the "empty" color -- no
// use of "inlier" or "outlier" pixel locations.
TEST_F(RenderEngineGlTest, NoBodyTest) {
  Init(RigidTransformd::Identity());
  Render();

  SCOPED_TRACE("NoBodyTest");
  VerifyUniformColor(kBgColor);
  VerifyUniformLabel(RenderLabel::kEmpty);
  VerifyUniformDepth(std::numeric_limits<float>::infinity());
}

// Confirm that the color image clear color gets successfully configured.
TEST_F(RenderEngineGlTest, ControlBackgroundColor) {
  std::vector<Rgba> backgrounds{Rgba{0.1, 0.2, 0.3, 1.0},
                                Rgba{0.5, 0.6, 0.7, 1.0},
                                Rgba{1.0, 0.1, 0.4, 0.5}};
  for (const auto& bg : backgrounds) {
    // TODO(SeanCurtis-TRI): When GCC supports non-trivial designated
    //  initialization turn this back into:
    //  const RenderEngineGlParams params{.default_clear_color = bg};
    RenderEngineGlParams params;
    params.default_clear_color = bg;
    RenderEngineGl engine(params);
    Render(&engine);
    VerifyUniformColor(bg, 0u);
  }
}

// Tests an image with *only* terrain (perpendicular to the camera's forward
// direction) -- no use of "inlier" or "outlier" pixel locations.
TEST_F(RenderEngineGlTest, TerrainTest) {
  Init(X_WR_, true);

  // At two different distances.
  Vector3d p_WR = X_WR_.translation();
  for (auto depth : std::array<float, 2>({{2.f, 4.9999f}})) {
    p_WR.z() = depth;
    X_WR_.set_translation(p_WR);
    renderer_->UpdateViewpoint(X_WR_);
    Render();
    SCOPED_TRACE(fmt::format("TerrainTest: depth = {}", depth));
    VerifyUniformDepth(depth);
    VerifyUniformLabel(RenderLabel::kDontCare);
  }

  {
    // Closer than kZNear.
    p_WR.z() = kZNear - 1e-5;
    X_WR_.set_translation(p_WR);
    renderer_->UpdateViewpoint(X_WR_);
    Render();
    SCOPED_TRACE("TerrainTest: ground closer than z-near");
    VerifyUniformDepth(ImageTraits<PixelType::kDepth32F>::kTooClose);
    VerifyUniformLabel(RenderLabel::kDontCare);
  }
  {
    // Farther than kZFar.
    p_WR.z() = kZFar + 1e-3;
    X_WR_.set_translation(p_WR);
    renderer_->UpdateViewpoint(X_WR_);
    Render();
    SCOPED_TRACE("TerrainTest: ground farther than z-far");
    VerifyUniformDepth(ImageTraits<PixelType::kDepth32F>::kTooFar);
    VerifyUniformLabel(RenderLabel::kDontCare);
  }
}

// Creates a terrain and then positions the camera such that a horizon between
// terrain and sky appears -- no use of "inlier" or "outlier" pixel locations.
TEST_F(RenderEngineGlTest, HorizonTest) {
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

  const RgbaColor bg_color(kBgColor);
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
      if ((static_cast<uint8_t>(bg_color.r != color.at(0, y)[0])) ||
          (static_cast<uint8_t>(bg_color.g != color.at(0, y)[1])) ||
          (static_cast<uint8_t>(bg_color.b != color.at(0, y)[2]))) {
        actual_horizon = y;
        break;
      }
    }

    const double expected_horizon = CalcHorizon(z);
    ASSERT_NEAR(expected_horizon, actual_horizon, 1.001) << "z = " << z;
  }
}

// Performs the shape-centered-in-the-image test with a box.
TEST_F(RenderEngineGlTest, BoxTest) {
  const auto& intrinsics = depth_camera_.core().intrinsics();
  for (const bool use_texture : {false, true}) {
    // Note the scale factors of 0.5 and 1.5 were selected so that the corner
    // of the box that is being tested will have the same color. Once because
    // half the texture is stretched over the face and once such that 1.5
    // iterations get stretched over the face.
    for (const double texture_scale : {1.0, 1.5, 0.5}) {
      const bool texture_scaled = texture_scale != 1;
      // We only need to sample texture scale if we're *using* the texture.
      if (!use_texture && texture_scaled) continue;
      Init(X_WR_, true);

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
      // the un-tiled default behavior and the ability to scale the texture.
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

        // TODO(SeanCurtis-TRI) This isn't *exactly* the same color as in the
        //  VTK test (130, 119, 16). I need to figure out where this color
        //  discrepancy comes from. The VTK test has less red and more green.
        //  That means the gl test is evaluating a texture coordinate that is
        //  *farther* from the origin than the VTK. Why?
        //  It's worth noting, that when setting the scale factor to 1.5 in the
        //  VTK test, the resulting color is consistent with this color (i.e.,
        //  within 1/255 on each channel). However, as I increase the scale
        //  factor from 1.5 to 3.5 to 10.5, the observed color stretches further
        //  into the red. This is clearly not an overly precise test.
        expected_color_ = RgbaColor(ColorI{135, 116, 15}, 255);
        // Quick proof that we're testing for a different color -- we're drawing
        // the red channel from our expected color.
        ASSERT_NE(kTextureColor.r(), expected_color_.r);
      } else {
        // Otherwise the expected is simply the texture color of box.png.
        expected_color_ =
            RgbaColor(use_texture ? kTextureColor : default_color_);
      }

      SCOPED_TRACE(fmt::format(
          "Box test - {} - scale {}",
          use_texture ? "texture" : "diffuse color",
          texture_scale));
      PerformCenterShapeTest(renderer_.get());
    }
  }
}

// Performs the shape centered in the image with a sphere.
TEST_F(RenderEngineGlTest, SphereTest) {
  for (const bool use_texture : {false, true}) {
    Init(X_WR_, true);

    PopulateSphereTest(renderer_.get(), use_texture);
    expected_color_ = RgbaColor(use_texture ? kTextureColor : default_color_);

    SCOPED_TRACE(
        fmt::format("Sphere test {}", use_texture ? "textured" : "rgba"));
    PerformCenterShapeTest(renderer_.get());
  }
}

// Performs the shape-centered-in-the-image test with a transparent sphere.
TEST_F(RenderEngineGlTest, TransparentSphereTest) {
  RenderEngineGl renderer;
  InitializeRenderer(X_WR_, true /* add terrain */, &renderer);
  const int int_alpha = 128;
  // Sets the color of the sphere that will be created in PopulateSphereTest.
  default_color_ = Rgba(kDefaultVisualColor.r(), kDefaultVisualColor.g(),
                        kDefaultVisualColor.b(), int_alpha / 255.0);
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
  auto blend = [](const Rgba& c1, const Rgba& c2, double alpha) {
    const double r = c1.r() * alpha + c2.r() * (1 - alpha);
    const double g = c1.g() * alpha + c2.g() * (1 - alpha);
    const double b = c1.b() * alpha + c2.b() * (1 - alpha);
    const double a = std::max(c1.a(), c2.a());
    return Rgba{r, g, b, a};
  };
  const double linear_factor = int_alpha / 255.0;
  const RgbaColor expect_linear{
      blend(kDefaultVisualColor, kTerrainColor, linear_factor)};
  const double quad_factor = linear_factor * (-linear_factor + 2);
  const RgbaColor expect_quad{
      blend(kDefaultVisualColor, kTerrainColor, quad_factor)};

  const ScreenCoord inlier = GetInlier(intrinsics);
  const RgbaColor at_pixel(color.at(inlier.x, inlier.y));
  EXPECT_TRUE(CompareColor(expect_linear, color, inlier) ||
              CompareColor(expect_quad, color, inlier))
      << "Expected one of two colors:"
      << "\n  " << expect_linear << " or " << expect_quad << "."
      << "\n  Found " << at_pixel;
}

// Performs the shape-centered-in-the-image test with a capsule.
TEST_F(RenderEngineGlTest, CapsuleTest) {
  Init(X_WR_, true);

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

  SCOPED_TRACE("Capsule test");
  PerformCenterShapeTest(renderer_.get());
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
TEST_F(RenderEngineGlTest, CapsuleRotatedTest) {
  Init(X_WR_, true);

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

  SCOPED_TRACE("Capsule rotated test");
  VerifyOutliers(*renderer_, depth_camera_);

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
        << "Color at: " << offset_inlier;
    EXPECT_TRUE(IsExpectedDepth(depth_, offset_inlier, expected_object_depth_,
                                kDepthTolerance))
        << "Depth at: " << offset_inlier;
    EXPECT_EQ(label_.at(x, y)[0], static_cast<int>(expected_label_))
        << "Label at: " << offset_inlier;
  }
}

// Performs the shape centered in the image with a cylinder.
TEST_F(RenderEngineGlTest, CylinderTest) {
  for (const bool use_texture : {false, true}) {
    Init(X_WR_, true);

    // Sets up a cylinder.
    Cylinder cylinder(0.2, 1.2);
    expected_label_ = RenderLabel(2);
    const GeometryId id = GeometryId::get_new_id();
    renderer_->RegisterVisual(id, cylinder, simple_material(use_texture),
                              RigidTransformd::Identity(),
                              true /* needs update */);
    // Position the top of the cylinder to be 1 m above the terrain.
    RigidTransformd X_WV{Translation3d(0, 0, 0.4)};
    renderer_->UpdatePoses(
        unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});

    SCOPED_TRACE("Cylinder test");
    expected_color_ = RgbaColor(use_texture ? kTextureColor : default_color_);
    PerformCenterShapeTest(renderer_.get());
  }
}

// Performs the shape-centered-in-the-image test with an ellipsoid rotated
// three different ways for confirming each extent axis.
TEST_F(RenderEngineGlTest, EllipsoidTest) {
  Init(X_WR_, true);

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
  {
    SCOPED_TRACE("Ellipsoid test: c extent");
    PerformCenterShapeTest(renderer_.get());
  }

  // Rotate the ellipsoid so that the 'b' extent is aligned with the z-axis of
  // the world, then move it by (target_z - b) units along the z-axis.
  X_WV =
      RigidTransformd{RotationMatrixd{AngleAxisd(-M_PI / 2, Vector3d::UnitX())},
                      Vector3d{0, 0, target_z - b}};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});
  {
    SCOPED_TRACE("Ellipsoid test: b extent");
    PerformCenterShapeTest(renderer_.get());
  }

  // Rotate the ellipsoid so that the 'a' extent is aligned with the z-axis of
  // the world, then move it by (target_z - a) units along the z-axis.
  X_WV =
      RigidTransformd{RotationMatrixd{AngleAxisd(M_PI / 2, Vector3d::UnitY())},
                      Vector3d{0, 0, target_z - a}};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});
  {
    SCOPED_TRACE("Ellipsoid test: a extent");
    PerformCenterShapeTest(renderer_.get());
  }
}

// Performs the shape centered in the image with a mesh (which happens to be a
// box). This simultaneously confirms that if a diffuse_map is specified but it
// doesn't refer to a file that can be read, that the appearance defaults to
// the diffuse rgba value.
TEST_F(RenderEngineGlTest, MeshTest) {
  Init(X_WR_, true);

  auto filename =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj");
  Mesh mesh(filename);
  expected_label_ = RenderLabel(3);
  PerceptionProperties material = simple_material();
  // NOTE: Specifying a diffuse map with a known bad path, will force the box
  // to get the diffuse RGBA value (otherwise it would pick up the `box.png`
  // texture).
  material.AddProperty("phong", "diffuse_map", "bad_path");
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, mesh, material, RigidTransformd::Identity(),
                            true /* needs update */);
  renderer_->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
      {id, RigidTransformd::Identity()}});

  SCOPED_TRACE("Mesh test");
  PerformCenterShapeTest(renderer_.get());
}

// Performs the shape-centered-in-the-image test with a *textured* mesh (which
// happens to be a box).
TEST_F(RenderEngineGlTest, TextureMeshTest) {
  Init(X_WR_, true);

  auto filename =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj");
  Mesh mesh(filename);
  expected_label_ = RenderLabel(4);
  PerceptionProperties material = simple_material();
  material.AddProperty(
      "phong", "diffuse_map",
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.png"));
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, mesh, material, RigidTransformd::Identity(),
                            true /* needs update */);
  renderer_->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
      {id, RigidTransformd::Identity()}});

  {
    SCOPED_TRACE("Textured mesh test");
    expected_color_ = RgbaColor(kTextureColor);
    PerformCenterShapeTest(renderer_.get());
  }

  {
    SCOPED_TRACE("Clone textured mesh test");
    // Now confirm that the texture survives cloning.
    unique_ptr<RenderEngine> clone = renderer_->Clone();
    RenderEngineGl* gl_engine = dynamic_cast<RenderEngineGl*>(clone.get());
    EXPECT_NE(gl_engine, nullptr);
    PerformCenterShapeTest(gl_engine);
  }
}

// TODO(SeanCurtis-TRI) Enable this test when I support the default logic for
//  mapping mesh.obj --> mesh.png.

// Repeat the texture test but with an *implied* texture map. In other words,
// registering a mesh "foo.obj" will look for a "foo.png" in the same folder as
// a fall back and use it if found. But *only* as a back up. This is a
// SHORT TERM hack to get textures in.
TEST_F(RenderEngineGlTest, ImpliedTextureMeshTest) {
  Init(X_WR_, true);

  auto filename =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj");
  Mesh mesh(filename);
  expected_label_ = RenderLabel(4);
  PerceptionProperties material = simple_material();
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, mesh, material, RigidTransformd::Identity(),
                            true /* needs update */);
  renderer_->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
      {id, RigidTransformd::Identity()}});

  expected_color_ = RgbaColor(kTextureColor);
  SCOPED_TRACE("Implied textured mesh test");
  PerformCenterShapeTest(renderer_.get());
}

// Performs the shape centered in the image with a convex mesh (which happens to
// be a  box). This simultaneously confirms that if a diffuse_map is specified
// but it doesn't refer to a file that can be read, that the appearance defaults
// to the diffuse rgba value.
TEST_F(RenderEngineGlTest, ConvexTest) {
  Init(X_WR_, true);

  auto filename = FindResourceOrThrow(
      "drake/geometry/render/test/meshes/box.obj");
  Convex convex(filename);
  expected_label_ = RenderLabel(4);
  PerceptionProperties material = simple_material();
  // NOTE: Specifying a diffuse map with a known bad path, will force the box
  // to get the diffuse RGBA value (otherwise it would pick up the `box.png`
  // texture.
  material.AddProperty("phong", "diffuse_map", "bad_path");
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, convex, material, RigidTransformd::Identity(),
                            true /* needs update */);
  renderer_->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
      {id, RigidTransformd::Identity()}});

  SCOPED_TRACE("Mesh test");
  PerformCenterShapeTest(renderer_.get());
}

// This confirms that geometries are correctly removed from the render engine.
// We add two new geometries (testing the rendering after each addition).
// By removing the first of the added geometries, we can confirm that the
// remaining geometries are re-ordered appropriately. Then, by removing the
// second, we should restore the original default image.
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
TEST_F(RenderEngineGlTest, RemoveVisual) {
  Init(X_WR_, true);
  PopulateSphereTest(renderer_.get());
  RenderLabel default_label = expected_label_;
  const float default_depth = expected_object_depth_;
  const double kRadius = 0.5;

  // Positions a sphere centered at <0, 0, z> with the given color.
  auto add_sphere = [this, kRadius](double z, GeometryId geometry_id) {
    const Sphere sphere{kRadius};
    const float depth = kDefaultDistance - kRadius - z;
    RenderLabel label = RenderLabel(5);
    PerceptionProperties material;
    material.AddProperty("label", "id", label);
    material.AddProperty("phong", "diffuse", kDefaultVisualColor);
    // This will accept all registered geometries and therefore, (bool)index
    // should always be true.
    renderer_->RegisterVisual(geometry_id, sphere, material,
                              RigidTransformd::Identity(), true);
    const RigidTransformd X_WV{Translation3d(0, 0, z)};
    X_WV_.insert({geometry_id, X_WV});
    renderer_->UpdatePoses(X_WV_);
    return std::make_tuple(label, depth);
  };

  // Sets the expected values prior to calling PerformCenterShapeTest().
  auto set_expectations = [this](const float depth, const RenderLabel& label) {
    expected_label_ = label;
    expected_object_depth_ = depth;
  };

  // We'll be adding and removing two spheres; get their ids ready.
  const GeometryId id1 = GeometryId::get_new_id();
  const GeometryId id2 = GeometryId::get_new_id();

  {
    // Add another sphere of a different color in front of the default sphere.
    // Desired position of the *top* of the sphere is z₁ = 1.5.
    float depth{};
    RenderLabel label{};
    std::tie(label, depth) = add_sphere(1.5 - kRadius, id1);
    set_expectations(depth, label);
    SCOPED_TRACE("Second sphere added in remove test");
    PerformCenterShapeTest(renderer_.get());
  }

  {
    // Add a _third_ sphere in front of the second.
    // Desired position of the *top* of the sphere is z₂ = 2.0.
    float depth{};
    RenderLabel label{};
    std::tie(label, depth) = add_sphere(2.0 - kRadius, id2);
    set_expectations(depth, label);
    SCOPED_TRACE("Third sphere added in remove test");
    PerformCenterShapeTest(renderer_.get());
  }

  {
    // Remove the first sphere added; should report "true" and the render test
    // should pass without changing expectations.
    const bool removed = renderer_->RemoveGeometry(id1);
    EXPECT_TRUE(removed);
    SCOPED_TRACE("First added sphere removed");
    PerformCenterShapeTest(renderer_.get());
  }

  {
    // Remove the second added sphere; should report true and rendering should
    // return to its default configuration.
    const bool removed = renderer_->RemoveGeometry(id2);
    EXPECT_TRUE(removed);
    set_expectations(default_depth, default_label);
    SCOPED_TRACE("Default image restored by removing extra geometries");
    PerformCenterShapeTest(renderer_.get());
  }
}

// All of the clone tests use the PerformCenterShapeTest() with the sphere setup
// to confirm that the clone is behaving as anticipated.

// Tests that the cloned renderer produces the same images (i.e., passes the
// same test).
TEST_F(RenderEngineGlTest, SimpleClone) {
  Init(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  unique_ptr<RenderEngine> clone = renderer_->Clone();
  EXPECT_NE(dynamic_cast<RenderEngineGl*>(clone.get()), nullptr);
  SCOPED_TRACE("Simple clone");
  PerformCenterShapeTest(dynamic_cast<RenderEngineGl*>(clone.get()));
}

// Tests that the cloned renderer still works, even when the original is
// deleted.
TEST_F(RenderEngineGlTest, ClonePersistence) {
  Init(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  unique_ptr<RenderEngine> clone = renderer_->Clone();
  // This causes the original renderer copied from to be destroyed.
  renderer_.reset();
  ASSERT_EQ(nullptr, renderer_);
  SCOPED_TRACE("Clone persistence");
  PerformCenterShapeTest(static_cast<RenderEngineGl*>(clone.get()));
}

// Tests that the cloned renderer still works, even when the original has values
// changed.
TEST_F(RenderEngineGlTest, CloneIndependence) {
  Init(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  unique_ptr<RenderEngine> clone = renderer_->Clone();
  // Move the terrain *up* 10 units in the z.
  RigidTransformd X_WT_new{Translation3d{0, 0, 10}};
  // This assumes that the terrain is zero-indexed.
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{geometry_id_, X_WT_new}});
  SCOPED_TRACE("Clone independence");
  PerformCenterShapeTest(dynamic_cast<RenderEngineGl*>(clone.get()));
}

// Confirm that the renderer can be used for cameras with different properties.
// I.e., the camera intrinsics are defined *outside* the renderer.
TEST_F(RenderEngineGlTest, DifferentCameras) {
  Init(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  const auto& ref_core = depth_camera_.core();
  const std::string& ref_name = ref_core.renderer_name();
  const RigidTransformd ref_X_BS = ref_core.sensor_pose_in_camera_body();
  const ClippingRange& ref_clipping = ref_core.clipping();
  const auto& ref_intrinsics = ref_core.intrinsics();
  const int ref_w = ref_intrinsics.width();
  const int ref_h = ref_intrinsics.height();
  const double ref_fov_y = ref_intrinsics.fov_y();
  {
    // Baseline -- confirm that all of the defaults in this test still produce
    // the expected outcome.
    SCOPED_TRACE("Camera change - baseline");
    PerformCenterShapeTest(renderer_.get(), &depth_camera_);
  }

  // Test changes in sensor sizes.
  {
    // Now run it again with a camera with a smaller sensor (quarter area).
    const DepthRenderCamera small_camera{
        {ref_name, {ref_w / 2, ref_h / 2, ref_fov_y}, ref_clipping, ref_X_BS},
        depth_camera_.depth_range()};
    {
      SCOPED_TRACE("Camera change - small camera");
      PerformCenterShapeTest(renderer_.get(), &small_camera);
    }

    // Now run it again with a camera with a bigger sensor (4X area).
    {
      const DepthRenderCamera big_camera{
          {ref_name, {ref_w * 2, ref_h * 2, ref_fov_y}, ref_clipping, ref_X_BS},
          depth_camera_.depth_range()};
      SCOPED_TRACE("Camera change - big camera");
      PerformCenterShapeTest(renderer_.get(), &big_camera);
    }

    // Test rendering with a previous camera once again to make sure the
    // existing target is reused correctly despite being of different sizes.
    {
      SCOPED_TRACE("Camera change - small camera");
      PerformCenterShapeTest(renderer_.get(), &small_camera);
    }
  }

  // Test changes in fov (larger and smaller).
  {
    const DepthRenderCamera narrow_fov{
        {ref_name, {ref_w, ref_h, ref_fov_y / 2}, ref_clipping, ref_X_BS},
        depth_camera_.depth_range()};
    SCOPED_TRACE("Camera change - narrow fov");
    PerformCenterShapeTest(renderer_.get(), &narrow_fov);
  }

  {
    const DepthRenderCamera wide_fov{
        {ref_name, {ref_w, ref_h, ref_fov_y * 2}, ref_clipping, ref_X_BS},
        depth_camera_.depth_range()};
    SCOPED_TRACE("Camera change - wide fov");
    PerformCenterShapeTest(renderer_.get(), &wide_fov);
  }

  // Test changes to depth range.
  {
    const auto& depth_range = depth_camera_.depth_range();
    const DepthRenderCamera clipping_far_plane{
        depth_camera_.core(),
        {depth_range.min_depth(), expected_outlier_depth_ - 0.1}};
    const float old_outlier_depth = expected_outlier_depth_;
    expected_outlier_depth_ = std::numeric_limits<float>::infinity();
    {
      SCOPED_TRACE("Camera change - z far clips terrain");
      PerformCenterShapeTest(renderer_.get(), &clipping_far_plane);
      // NOTE: Need to restored expected outlier depth for next test.
      expected_outlier_depth_ = old_outlier_depth;
    }

    {
      const DepthRenderCamera clipping_near_plane{
        depth_camera_.core(),
        {expected_object_depth_ + 0.1, depth_range.max_depth()}};
      const float old_object_depth = expected_object_depth_;
      expected_object_depth_ = 0;
      SCOPED_TRACE("Camera change - z near clips mesh");
      PerformCenterShapeTest(renderer_.get(), &clipping_near_plane);
      // NOTE: Need to restored expected object depth for next test.
      expected_object_depth_ = old_object_depth;
    }

    {
      // Test rendering with a previous camera once again to make sure the
      // existing target is reused correctly.
      expected_outlier_depth_ = std::numeric_limits<float>::infinity();
      SCOPED_TRACE("Camera change - z far clips terrain 2");
      PerformCenterShapeTest(renderer_.get(), &clipping_far_plane);
    }
  }
}

// Tests that registered geometry without any explicitly set perception
// properties renders without error.
// TODO(SeanCurtis-TRI): When RenderEngineGl supports label and color images,
// default values for the geometry *appearance* will be available (and
// settable). This test should extend to confirm that the default values are
// used in the absence of explicitly set geometry properties, and that changing
// the engine's default values likewise get exercised here.
// But, for depth, for now, simply confirming that it renders is sufficient.
TEST_F(RenderEngineGlTest, DefaultProperties) {
  Init(X_WR_, false /* no terrain */);

  // Sets up a box.
  Box box(1, 1, 1);
  const GeometryId id = GeometryId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      renderer_->RegisterVisual(id, box, PerceptionProperties(),
                                RigidTransformd::Identity(), true),
      std::logic_error,
      ".* geometry with the 'unspecified' or 'empty' render labels.*");
  PerceptionProperties material;
  material.AddProperty("label", "id", RenderLabel::kDontCare);
  renderer_->RegisterVisual(id, box, material, RigidTransformd::Identity(),
                            true);
  RigidTransformd X_WV{Translation3d(0, 0, 0.5)};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});

  EXPECT_NO_THROW(Render());
}

// Tests the ability to configure the RenderEngineGl's default render label.
TEST_F(RenderEngineGlTest, DefaultProperties_RenderLabel) {
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
    RenderEngineGl renderer;
    InitializeRenderer(X_WR_, false /* no terrain */, &renderer);

    DRAKE_EXPECT_THROWS_MESSAGE(
        populate_default_sphere(&renderer),
        std::logic_error,
        ".* geometry with the 'unspecified' or 'empty' render labels.*");
  }

  // Case: Change render engine's default to explicitly be unspecified; must
  // throw.
  {
    RenderEngineGl renderer{{.default_label = RenderLabel::kUnspecified}};
    InitializeRenderer(X_WR_, false /* no terrain */, &renderer);

    DRAKE_EXPECT_THROWS_MESSAGE(
        populate_default_sphere(&renderer),
        std::logic_error,
        ".* geometry with the 'unspecified' or 'empty' render labels.*");
  }

  // Case: Change render engine's default to don't care. Label image should
  // report don't care.
  {
    ResetExpectations();
    RenderEngineGl renderer{{.default_label = RenderLabel::kDontCare}};
    InitializeRenderer(X_WR_, true /* no terrain */, &renderer);

    DRAKE_EXPECT_NO_THROW(populate_default_sphere(&renderer));
    expected_label_ = RenderLabel::kDontCare;
    expected_color_ = RgbaColor(renderer.parameters().default_diffuse);

    SCOPED_TRACE("Default properties; don't care label");
    PerformCenterShapeTest(&renderer);
  }

  // Case: Change render engine's default to invalid default value; must throw.
  {
    for (RenderLabel label :
        {RenderLabel::kEmpty, RenderLabel(1), RenderLabel::kDoNotRender}) {
      DRAKE_EXPECT_THROWS_MESSAGE(
          RenderEngineGl({.default_label = label}), std::logic_error,
          ".* default render label .* either 'kUnspecified' or 'kDontCare'.*");
    }
  }
}

// Tests to see if the two images are *exactly* equal - down to the last bit.
::testing::AssertionResult ImagesExactlyEqual(const ImageDepth32F& ref,
                                              const ImageDepth32F& test) {
  if (ref.size() != test.size()) {
    return ::testing::AssertionFailure()
           << "Image sizes don't match\n"
           << "  ref image:  (" << ref.width() << ", " << ref.height() << ")\n"
           << "  test image: (" << test.width() << ", " << test.height() << ")";
  }
  const int byte_count = ref.size() * sizeof(float);
  if (std::memcmp(ref.at(0, 0), test.at(0, 0), byte_count) != 0) {
    return ::testing::AssertionFailure() << "Image contents don't match";
  }
  return ::testing::AssertionSuccess();
}

// This is a test to make sure that independent renders are truly independent.
// We want to make sure that setting OpenGL state in one render engine's OpenGL
// context doesn't pollute the state of the other render engine's OpenGL
// context. So, we're going to do a bunch of interleaved operations on the two
// renderers to make sure that each is still correct.
TEST_F(RenderEngineGlTest, RendererIndependence) {
  RenderEngineGl engine1;
  RenderEngineGl engine2;
  const GeometryId ground = GeometryId::get_new_id();

  auto add_terrain = [ground](auto* renderer) {
    PerceptionProperties material;
    material.AddProperty("label", "id", RenderLabel::kDontCare);
    renderer->RegisterVisual(ground, HalfSpace(), material,
                             RigidTransformd::Identity(),
                             false /* needs update */);
  };

  // Note: All invocations of Render() renders to the member image: depth_.
  engine1.UpdateViewpoint(X_WR_);
  add_terrain(&engine1);
  PopulateSphereTest(&engine1);
  Render(&engine1);
  // We'll store the resultant depth image to compare with subsequent rendering.
  const ImageDepth32F reference_1(depth_);

  // Create a depth image containing nothing but infinity; it *must* be
  // different from the engine 1 original image. This serves as a reality check.
  Render(&engine2);
  VerifyUniformDepth(std::numeric_limits<float>::infinity());
  const ImageDepth32F empty_2(depth_);
  ASSERT_FALSE(ImagesExactlyEqual(reference_1, empty_2));

  // Re-render engine 1 (unchanged). The resultant image should still match
  // the reference image.
  Render(&engine1);
  ASSERT_TRUE(ImagesExactlyEqual(reference_1, depth_));

  // Populate engine 2 with a box. It's depth image must be different from all
  // previous images.
  engine2.UpdateViewpoint(X_WR_);
  add_terrain(&engine2);
  Box box(1, 1, 1);
  const GeometryId box_id = GeometryId::get_new_id();
  engine2.RegisterVisual(box_id, box, simple_material(),
                         RigidTransformd::Identity(), true);
  RigidTransformd X_WV{Translation3d(0.2, 0.2, 0.5)};
  engine2.UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{box_id, X_WV}});
  Render(&engine2);
  ASSERT_FALSE(ImagesExactlyEqual(reference_1, depth_));
  ASSERT_FALSE(ImagesExactlyEqual(empty_2, depth_));

  // Re-render engine 1 (unchanged). The resultant image should still match
  // the reference image.
  Render(&engine1);
  ASSERT_TRUE(ImagesExactlyEqual(reference_1, depth_));
}

// Confirms that passing in show_window = true "works". The test can't confirm
// that a window appears that a human would see. Instead, it confirms that
// RenderEngineGl is exercising the OpenGlContext responsible for window
// management and infers from that, that window display works.
TEST_F(RenderEngineGlTest, ShowRenderLabel) {
  RenderEngineGl engine;
  const internal::OpenGlContext& context =
      RenderEngineGlTester(&engine).opengl_context();
  const int w = depth_camera_.core().intrinsics().width();
  const int h = depth_camera_.core().intrinsics().height();
  ImageLabel16I image{w, h};

  ASSERT_FALSE(context.IsWindowViewable());
  engine.RenderLabelImage(ColorRenderCamera{depth_camera_.core(), true},
                          &image);
  ASSERT_TRUE(context.IsWindowViewable());
  engine.RenderLabelImage(ColorRenderCamera{depth_camera_.core(), false},
                          &image);
  ASSERT_FALSE(context.IsWindowViewable());

  // TODO(SeanCurtis-TRI): Do the same for color labels when implemented.
}

// Confirms that when requesting the same mesh multiple times, only a single
// OpenGlGeometry is produced.
TEST_F(RenderEngineGlTest, MeshGeometryReuse) {
  RenderEngineGl engine;
  RenderEngineGlTester tester(&engine);

  auto filename = FindResourceOrThrow(
      "drake/geometry/render/test/meshes/box.obj");
  const internal::OpenGlGeometry& initial_geometry = tester.GetMesh(filename);
  const internal::OpenGlGeometry& second_geometry = tester.GetMesh(filename);

  EXPECT_EQ(initial_geometry.vertex_array, second_geometry.vertex_array);
  EXPECT_EQ(initial_geometry.vertex_buffer, second_geometry.vertex_buffer);
  EXPECT_EQ(initial_geometry.index_buffer, second_geometry.index_buffer);
  EXPECT_EQ(initial_geometry.index_buffer_size,
            second_geometry.index_buffer_size);
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
  const RgbaColor ground(kTerrainColor);
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
      const AdjacentPixel bottom_result =
          Compare(curr_pixel, bottom_pixel);
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
      const AdjacentPixel right_result =
          Compare(curr_pixel, right_pixel);
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
TEST_F(RenderEngineGlTest, IntrinsicsAndRenderProperties) {
  // TODO(#11965) Now that the creation of the projection matrix is part
  //  of RenderCameraCore (and tested there), this could be simplified. We rely
  //  on the *correctness* of the projection matrix and merely confirm that
  //  it is being computed at all. That would eliminate the many tweaks and
  //  comparisons. Consider simplifying this when the render engine test
  //  infrastructure is refactored.
  Init(X_WR_, true /* add_terrain */);
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
  ASSERT_TRUE((ref_box_edges.array() > -1).all()) << ref_box_edges.transpose();
  const int ref_box_width = ref_box_edges(2) - ref_box_edges(0);
  const int ref_box_height = ref_box_edges(1) - ref_box_edges(3);
  ASSERT_EQ(ref_box_width, ref_box_height);
  ASSERT_EQ((ref_box_edges(2) + ref_box_edges(0)) / 2, w / 2);
  ASSERT_EQ((ref_box_edges(1) + ref_box_edges(3)) / 2, h / 2);

  {
    // Also confirm the box is positioned the same in color and label images.
    const Vector4<int> color_edges = FindBoxEdges(ref_color);
    ASSERT_EQ(color_edges, ref_box_edges) << color_edges;
    const Vector4<int> label_edges = FindBoxEdges(ref_label);
    ASSERT_EQ(label_edges, ref_box_edges) << label_edges;
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
    ASSERT_TRUE((test_edges.array() > -1).all()) << test_edges.transpose();
    const int test_box_width = test_edges(2) - test_edges(0);
    const int test_box_height = test_edges(1) - test_edges(3);

    // Confirm the box gets squished.
    const double fx_ratio = fx2 / fx;
    const double fy_ratio = fy2 / fy;
    EXPECT_NEAR(test_box_width, ref_box_width * fx_ratio, 1);
    EXPECT_NEAR(test_box_height, ref_box_height * fy_ratio, 1);

    // Confirm that its center is translated.
    EXPECT_NEAR((test_edges(0) + test_edges(2)) / 2.0, w2 / 2.0 + offset_x, 1.0)
        << test_edges.transpose();
    EXPECT_NEAR((test_edges(1) + test_edges(3)) / 2.0, h2 / 2.0 + offset_y, 1.0)
        << test_edges.transpose();

    {
      // Also confirm it matches for color and label.
      const Vector4<int> color_edges = FindBoxEdges(color);
      ASSERT_EQ(color_edges, test_edges) << color_edges.transpose();
      const Vector4<int> label_edges = FindBoxEdges(label);
      ASSERT_EQ(label_edges, test_edges) << label_edges.transpose();
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
    VerifyUniformColor(kBgColor, &color);
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
    VerifyUniformColor(kBgColor, &color);
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
    EXPECT_TRUE(
        IsExpectedDepth(depth, ScreenCoord{w / 2, h / 2},
            ImageTraits<PixelType::kDepth32F>::kTooClose, 0.0));
    EXPECT_TRUE(
        IsExpectedDepth(depth, ScreenCoord{0, 0},
            ImageTraits<PixelType::kDepth32F>::kTooFar, 0.0));
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
    EXPECT_TRUE(
        IsExpectedDepth(depth, ScreenCoord{w / 2, h / 2},
            ImageTraits<PixelType::kDepth32F>::kTooFar, 0.0));
    EXPECT_TRUE(
        IsExpectedDepth(depth, ScreenCoord{0, 0},
            ImageTraits<PixelType::kDepth32F>::kTooFar, 0.0));
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
    EXPECT_TRUE(
        IsExpectedDepth(depth, ScreenCoord{w / 2, h / 2},
            ImageTraits<PixelType::kDepth32F>::kTooClose, 0.0));
    EXPECT_TRUE(
        IsExpectedDepth(depth, ScreenCoord{0, 0},
            ImageTraits<PixelType::kDepth32F>::kTooClose, 0.0));
  }
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
