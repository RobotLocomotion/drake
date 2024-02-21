#include "drake/geometry/render_vtk/internal_render_engine_vtk.h"

#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <variant>
#include <vector>

#include <Eigen/Dense>
#include <fmt/format.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkOpenGLTexture.h>  // vtkRenderingOpenGL2
#include <vtkPNGReader.h>      // vtkIOImage
#include <vtkProperty.h>       // vtkRenderingCore

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
#include "drake/systems/sensors/image_io.h"
#include "drake/systems/sensors/test_utilities/image_compare.h"
#include "drake/visualization/colorize_depth_image.h"

/* Note: enabling this causes failures with two tests. Try running as:

 bazel test //geometry/render_vtk:internal_render_engine_vtk_test \
    --test_filter=-*DifferentCameras:*Intrinsics*

 to get past the aberrant tests; they *should* pass with this disabled. */
DEFINE_bool(show_window, false, "Display render windows locally for debugging");
DEFINE_double(sleep, 0, "Seconds to sleep between renders");

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {

// Use friend access to grab actors.
class RenderEngineVtkTester {
 public:
  // This returns the first color actor associated with the given `id` (if there
  // are multiple actors for the geometry).
  static vtkActor* GetColorActor(const RenderEngineVtk& renderer,
                                 GeometryId id) {
    // First 0 is the color index, second is the first actor.
    vtkActor* actor = renderer.props_.at(id).at(0).parts.at(0).actor.Get();
    DRAKE_DEMAND(actor != nullptr);
    return actor;
  }

  // Return all of the colors actors associated with the given geometry id.
  static std::vector<vtkActor*> GetColorActors(const RenderEngineVtk& renderer,
                                               GeometryId id) {
    const auto& color_prop = renderer.props_.at(id).at(0);
    std::vector<vtkActor*> actors;
    for (const auto& part : color_prop.parts) {
      actors.push_back(part.actor.Get());
    }
    return actors;
  }
};

namespace {

using Eigen::AngleAxisd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransformd;
using math::RotationMatrixd;
using render::ColorRenderCamera;
using render::DepthRange;
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
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageGrey8U;
using systems::sensors::ImageIo;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::ImageTraits;
using systems::sensors::PixelType;
using visualization::ColorizeDepthImage;

// Default camera properties.
const int kWidth = 640;
const int kHeight = 480;
const double kClipNear = 0.1;
const double kClipFar = 100.0;
const double kZNear = 0.5;
const double kZFar = 5.;
const double kFovY = M_PI_4;

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

// An RGBA color denoted using four `int`s, offering nice conversion
// constructors and operators to ease the pain of creating test values.
struct TestColor {
  // Constructs from three or four `int`s.
  constexpr TestColor(int r_in, int g_in, int b_in, int a_in = 255)
      : r(r_in), g(g_in), b(b_in), a(a_in) {}

  // Constructs from an array of four bytes.
  explicit TestColor(const uint8_t* p) : r(p[0]), g(p[1]), b(p[2]), a(p[3]) {}

  // Constructs from a vector of four doubles (each in the range [0..1]).
  explicit TestColor(const Vector4d& norm_color)
      : r(static_cast<int>(norm_color(0) * 255)),
        g(static_cast<int>(norm_color(1) * 255)),
        b(static_cast<int>(norm_color(2) * 255)),
        a(static_cast<int>(norm_color(3) * 255)) {}

  // This implicit conversion is extremely convenient.
  // NOLINTNEXTLINE(runtime/explicit)
  TestColor(const Rgba& rgba) : TestColor(rgba.rgba()) {}

  // Converts back to an Rgba.
  Rgba ToRgba() const {
    return Rgba(r / 255.0, g / 255.0, b / 255.0, a / 255.0);
  }

  bool operator==(const TestColor& c) const {
    return r == c.r && g == c.g && b == c.b && a == c.a;
  }

  bool operator!=(const TestColor& c) const { return !(*this == c); }

  int r{0};
  int g{0};
  int b{0};
  int a{255};
};

std::ostream& operator<<(std::ostream& out, const TestColor& c) {
  out << "(" << c.r << ", " << c.g << ", " << c.b << ", " << c.a << ")";
  return out;
}

// Background (sky) and terrain colors.
constexpr TestColor kBgColor{254, 127, 0};

// We need a color that we can see the effects of illumination on.
constexpr TestColor kTerrainColor{127, 127, 153};

// box.png contains a single pixel with the color (4, 241, 33). If the image
// changes, the expected color would likewise have to change.
constexpr TestColor kTextureColor{4, 241, 33};

// Provide a default visual color for these tests -- it is intended to be
// different from the default color of the VTK render engine.
constexpr TestColor kDefaultVisualColor{229, 229, 229};

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

// Tests color within tolerance.
bool IsColorNear(const TestColor& expected, const TestColor& tested,
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
    const TestColor& expected, const ImageRgba8U& image, const ScreenCoord& p,
    double tolerance = kColorPixelTolerance) {
  TestColor tested(image.at(p.x, p.y));
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
    const ColorRenderCamera color_camera(depth_camera.core(),
                                         FLAGS_show_window);
    ImageRgba8U* color = color_out ? color_out : &color_;
    ImageDepth32F* depth = depth_out ? depth_out : &depth_;
    ImageLabel16I* label = label_out ? label_out : &label_;
    EXPECT_NO_THROW(renderer->RenderDepthImage(depth_camera, depth));
    EXPECT_NO_THROW(renderer->RenderLabelImage(color_camera, label));
    EXPECT_NO_THROW(renderer->RenderColorImage(color_camera, color));
    if (FLAGS_sleep > 0) sleep(FLAGS_sleep);
  }

  // Confirms that all pixels in the member color image have the same value.
  void VerifyUniformColor(const TestColor& pixel,
                          const ImageRgba8U* color = nullptr) {
    if (color == nullptr) color = &color_;
    for (int y = 0; y < color->height(); ++y) {
      for (int x = 0; x < color->width(); ++x) {
        ASSERT_TRUE(CompareColor(pixel, *color, ScreenCoord{x, y}));
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
    RenderEngineVtkParams params{{}, bg_rgb};
    renderer_ = make_unique<RenderEngineVtk>(params);
    InitializeRenderer(X_WR, add_terrain, renderer_.get());
    // Ensure that we truly have a non-default color.
    EXPECT_FALSE(IsColorNear(kDefaultVisualColor,
                             TestColor(renderer_->default_diffuse())));
  }

  // Tests that instantiate their own renderers can initialize their renderers
  // with this method.
  void InitializeRenderer(const RigidTransformd& X_WR, bool add_terrain,
                          RenderEngineVtk* engine) {
    engine->UpdateViewpoint(X_WR);

    if (add_terrain) {
      PerceptionProperties material;
      material.AddProperty("label", "id", RenderLabel::kDontCare);
      material.AddProperty("phong", "diffuse", kTerrainColor.ToRgba());
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
    expected_color_ = kDefaultVisualColor;
    expected_outlier_color_ = kTerrainColor;
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

  TestColor expected_color_{kDefaultVisualColor};
  TestColor expected_outlier_color_{kDefaultVisualColor};
  float expected_outlier_depth_{3.f};
  float expected_object_depth_{2.f};
  RenderLabel expected_label_;
  RenderLabel expected_outlier_label_{RenderLabel::kDontCare};
  TestColor default_color_{kDefaultVisualColor};

  // We store a reference depth camera; we can always derive a color camera
  // from it; they have the same intrinsics and we grab the global
  // FLAGS_show_window.
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

  VerifyUniformColor(kBgColor);
  VerifyUniformLabel(RenderLabel::kEmpty);
  VerifyUniformDepth(std::numeric_limits<float>::infinity());
}

// Confirm that the color image clear color gets successfully configured.
TEST_F(RenderEngineVtkTest, ControlBackgroundColor) {
  std::vector<TestColor> backgrounds{
      {10, 20, 30}, {128, 196, 255}, {255, 10, 40}};
  for (const auto& bg : backgrounds) {
    RenderEngineVtkParams params{
        {}, Vector3d{bg.r / 255., bg.g / 255., bg.b / 255.}};
    RenderEngineVtk engine(params);
    Render(&engine);
    VerifyUniformColor(bg);
  }
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

    expected_color_ = use_texture ? kTextureColor : default_color_;
    PerformCenterShapeTest(
        renderer_.get(),
        fmt::format("Mesh test {}", use_texture ? "textured" : "rgba").c_str());
  }
}

// A simple regression test to make sure that we are supporting all of the
// texture types that glTF supports. To that end, we have a special glTF file
// that we'll render and test the resulting image against a reference image.
//
// Changes to the camera pose, the glTF file being tested, or render camera
// intrinsics will require the reference image to be re-rendered. Simply save
// the image that is rendered by this test as the new reference (subject to
// visual inspection).
TEST_F(RenderEngineVtkTest, GltfTextureSupport) {
  const RotationMatrixd R_WC(math::RollPitchYawd(-M_PI / 2.5, 0, M_PI / 4));
  const RigidTransformd X_WC(R_WC,
                             R_WC * Vector3d(0, 0, -6) + Vector3d(0, 0, -0.15));
  Init(X_WC);

  PerceptionProperties material;
  material.AddProperty("label", "id", RenderLabel(1));
  const GeometryId id = GeometryId::get_new_id();
  const std::string filename = FindResourceOrThrow(
      "drake/geometry/render/test/meshes/fully_textured_pyramid.gltf");
  renderer_->RegisterVisual(id, Mesh(filename), material,
                            RigidTransformd::Identity(),
                            false /* needs update */);
  ImageRgba8U image(64, 64);
  const ColorRenderCamera camera(
      {"unused", {64, 64, kFovY / 2}, {0.01, 10}, {}}, FLAGS_show_window);
  renderer_->RenderColorImage(camera, &image);

  ImageRgba8U expected_image;
  const std::string ref_filename = FindResourceOrThrow(
      "drake/geometry/render/test/fully_textured_pyramid_rendered.png");
  systems::sensors::LoadImage(ref_filename, &expected_image);
  // We're testing to see if the images are *coarsely* equal. This accounts for
  // the differences in CI's rendering technology from a local GPU. The images
  // are deemed equivalent if 80% of the channel values are within 20 of the
  // reference color.
  ASSERT_EQ(expected_image.size(), image.size());
  Eigen::Map<VectorX<uint8_t>> data_expected(expected_image.at(0, 0),
                                             expected_image.size());
  Eigen::Map<VectorX<uint8_t>> data2(image.at(0, 0), image.size());
  const auto differences =
      (data_expected.cast<float>() - data2.cast<float>()).array().abs();
  const int num_acceptable = (differences <= 20).count();
  EXPECT_GE(num_acceptable / static_cast<float>(expected_image.size()), 0.8);
}

// A glTF file can either embed its assets as data URIs, or can use relative
// pathnames for the URI. It can use buffer views into a single asset buffer,
// or have assets in different files. We'll render a simple cube loaded from
// multiple different asset formats, and confirm that it looks the same in all
// cases. This test doesn't confirm that VTK does the _right_ thing with glTF
// files (for that, see WholeImageCustomParams), just that it treats the glTF
// asset formats uniformly.
TEST_F(RenderEngineVtkTest, GltfAssetFormats) {
  constexpr int kCount = 3;
  const std::array<std::string, kCount> filenames{
      FindResourceOrThrow("drake/geometry/render/test/meshes/cube1.gltf"),
      FindResourceOrThrow("drake/geometry/render/test/meshes/cube2.gltf"),
      FindResourceOrThrow("drake/geometry/render/test/meshes/cube3.gltf")};

  Init(X_WC_, true);

  std::array<ImageRgba8U, kCount> images;
  for (int i = 0; i < ssize(filenames); ++i) {
    // Add the i'th cube to the scene.
    const GeometryId id = GeometryId::get_new_id();
    renderer_->RegisterVisual(id, Mesh(filenames[i]), PerceptionProperties{},
                              RigidTransformd::Identity(), true);
    renderer_->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
        {id, RigidTransformd::Identity()}});

    // Render an image of it.
    images[i].resize(kWidth, kHeight);
    Render(nullptr, nullptr, &images[i], nullptr, nullptr);

    // Compare against the prior image.
    if (i > 0) {
      EXPECT_TRUE(images[i - i] == images[i]) << fmt::format(
          "The gltf_asset_formats_{}.png and gltf_asset_formats_{}.png should "
          "have been identical, but were not! Check the bazel-testlogs for "
          "the saved images",
          i - 1, i);
    }

    // Reset for the next cube.
    renderer_->RemoveGeometry(id);
  }

  // Save the images for offline inspection.
  if (const char* dir = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR")) {
    const std::filesystem::path out_dir(dir);
    for (int i = 0; i < ssize(images); ++i) {
      ImageIo{}.Save(images[i],
                     out_dir / fmt::format("gltf_asset_formats_{}.png", i));
    }
  }
}

// Primitives result in a geometry with a single Part. However, we can load
// meshes from .gltf or .obj files that will create multiple parts. The meshes
// in this test are conceptually identical: a cube with different colors on each
// face. We'll render the cube six times with different orientations to expose
// each colored face to the camera, and confirm the observed color.
//
// The glTF file has been structured to further test various glTF features,
// including:
//
//  1. Multiple nodes.
//  2. Multiple root nodes.
//  3. Empty nodes (with non-identity transforms).
//  4. Hierarchies.
//  5. Textures. The texture is not vertically symmetric; if the image is
//     applied to the mesh badly, the asymmetry will reveal that. VTK has
//     exhibited a penchant for flipping images upside down with no rhyme nor
//     reason, so it's important to test with an image that would reveal that
//     kind of bug.
//  6. Materials.
//  7. Single meshes with multiple materials.
//
// If all of that is processed correctly, we should get a cube with a different
// color on each face. We'll test for those colors.
//
// The obj features under test are a subset of the glTF features.
TEST_F(RenderEngineVtkTest, MultiMaterialObjects) {
  // The name of the face we expect presented to the camera, and the rotation
  // required to put it in front of the camera. We'll use the name to look up
  // the expected color.
  struct Face {
    std::string name;
    RotationMatrixd rotation;
  };

  const std::vector<std::string> filenames{
      FindResourceOrThrow("drake/geometry/render/test/meshes/rainbow_box.gltf"),
      FindResourceOrThrow("drake/geometry/render/test/meshes/rainbow_box.obj")};

  // The expected *illuminated* material color, keyed first by mesh extension
  // and then by face name.
  //
  // For the glTF, the material/texture colors are not exactly reproduced
  // because the lighting model associated with the PBR shader. For now, we
  // account for this by putting the observed color in the test. If we change
  // the glTF (or lighting model), we'll need to update these values
  // accordingly.
  //
  // For the obj, it should be a reproduction of the diffuse color in the
  // .mtl file (where there is no texture) or the product of texture color
  // and Kd value. The red, green, and blue faces all share a common textured
  // material with the Kd value of (0.8, 0.8, 0.8). In the map below, the first
  // Rgba color represents the texture value, the second, the Kd value.
  const std::map<std::string, std::map<std::string, Rgba>> rendered_color{
      {".obj",
       {{"green", Rgba(0.016, 0.945, 0.129) * Rgba(0.8, 0.8, 0.8)},
        {"orange", Rgba(0.8, 0.359, 0.023)},
        {"red", Rgba(0.945, 0.016, 0.016) * Rgba(0.8, 0.8, 0.8)},
        {"blue", Rgba(0.098, 0.016, 0.945) * Rgba(0.8, 0.8, 0.8)},
        {"yellow", Rgba(0.799, 0.8, 0)},
        {"purple", Rgba(0.436, 0, 0.8)}}},
      {".gltf",
       {{"green", Rgba(0.078, 0.553, 0.110)},
        {"orange", Rgba(0.529, 0.259, 0.125)},
        {"red", Rgba(0.553, 0.078, 0.078)},
        {"blue", Rgba(0.098, 0.078, 0.553)},
        {"yellow", Rgba(0.529, 0.529, 0.075)},
        {"purple", Rgba(0.310, 0.075, 0.529)}}}};

  const std::vector<Face> faces{
      {.name = "green", .rotation = RotationMatrixd()},
      {.name = "orange", .rotation = RotationMatrixd::MakeXRotation(M_PI / 2)},
      {.name = "red", .rotation = RotationMatrixd::MakeXRotation(M_PI)},
      {.name = "blue", .rotation = RotationMatrixd::MakeXRotation(-M_PI / 2)},
      {.name = "yellow", .rotation = RotationMatrixd::MakeYRotation(-M_PI / 2)},
      {.name = "purple", .rotation = RotationMatrixd::MakeYRotation(M_PI / 2)},
  };

  for (const auto& filename : filenames) {
    Init(X_WC_, true);
    Mesh mesh(filename);
    // When we add a glTF file, the terrain's material color gets promoted to
    // PBR (to match). Therefore, the expected outlier color needs to shift
    // to account for the material change.
    expected_outlier_color_ = mesh.extension() == ".gltf"
                                  ? TestColor(Rgba(0.4392, 0.4392, 0.4745))
                                  : kTerrainColor;
    expected_label_ = RenderLabel(3);
    // Note: Passing diffuse color or texture to a glTF spawns a warning.
    PerceptionProperties material;
    material.AddProperty("label", "id", expected_label_);
    const GeometryId id = GeometryId::get_new_id();
    renderer_->RegisterVisual(id, mesh, material, RigidTransformd::Identity(),
                              true /* needs update */);

    // Render from the original to make sure it's complete and correct.
    for (const auto& face : faces) {
      expected_color_ = rendered_color.at(mesh.extension()).at(face.name);

      renderer_->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
          {id, RigidTransformd(face.rotation)}});
      PerformCenterShapeTest(renderer_.get(),
                             fmt::format("{} test on {} face - original",
                                         mesh.extension(), face.name)
                                 .c_str());
    }

    // Repeat that from a clone to confirm that the artifacts survived cloning.
    std::unique_ptr<RenderEngine> clone = renderer_->Clone();
    RenderEngineVtk* vtk_clone = dynamic_cast<RenderEngineVtk*>(clone.get());
    for (const auto& face : faces) {
      expected_color_ = rendered_color.at(mesh.extension()).at(face.name);

      vtk_clone->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
          {id, RigidTransformd(face.rotation)}});
      PerformCenterShapeTest(
          vtk_clone,
          fmt::format("{} test on {} face - clone", mesh.extension(), face.name)
              .c_str());
    }
  }
}

// When VTK imports a glTF file, the transforms of nodes in the file's frame are
// not stored in vtkProp3D's transform components (position, origin,
// orientation, and scale). This simply confirms that each of those quantities
// are the identity value.
bool TransformComponentsAreIdentity(vtkActor* a) {
  Vector3d position, origin, orientation, scale;
  a->GetPosition(position.data());
  a->GetOrigin(origin.data());
  a->GetOrientation(orientation.data());
  a->GetScale(scale.data());
  return (position.array() == 0).all() && (origin.array() == 0).all() &&
         (orientation.array() == 0).all() && (scale.array() == 1).all();
}

// How Drake uses VTK to handle glTF files is predicated on an understanding on
// how vtkGLTFImporter creates pose information for glTF nodes. This test serves
// as a signal if VTK's handling of glTF nodes changes. See
// TransformComponentsAreIdentity().
TEST_F(RenderEngineVtkTest, VtkGltfBehavior) {
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
  for (vtkActor* actor :
       RenderEngineVtkTester::GetColorActors(*renderer_, id)) {
    ASSERT_TRUE(TransformComponentsAreIdentity(actor));
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
  const ColorRenderCamera camera(depth_camera_.core(), FLAGS_show_window);
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
  TestColor default_color = expected_color_;
  RenderLabel default_label = expected_label_;
  float default_depth = expected_object_depth_;

  // Positions a sphere centered at <0, 0, z> with the given color.
  auto add_sphere = [this](const TestColor& diffuse, double z,
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
  auto set_expectations = [this](const TestColor& color, float depth,
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
  const TestColor color1(128, 128, 255, 255);
  float depth1{};
  RenderLabel label1{};
  const GeometryId id1 = GeometryId::get_new_id();
  std::tie(label1, depth1) = add_sphere(color1, 0.75, id1);
  set_expectations(color1, depth1, label1);
  PerformCenterShapeTest(renderer_.get(), "First sphere added in remove test");

  // Add a _third_ sphere in front of the second.
  const TestColor color2(128, 255, 128, 255);
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

// Tests that RenderEngineVtk's default render label is kDontCare.
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
  ResetExpectations();
  RenderEngineVtk renderer;
  InitializeRenderer(X_WC_, true /* add terrain */, &renderer);

  DRAKE_EXPECT_NO_THROW(populate_default_sphere(&renderer));
  expected_label_ = RenderLabel::kDontCare;
  expected_color_ = TestColor(renderer.default_diffuse());

  PerformCenterShapeTest(&renderer, "Default properties; don't care label");
}

// This class exists solely for the purpose of injecting an arbitrary texture
// onto an actor and confirm that the texture is preserved over the copy.
// For simplicity, we'll only register shapes that map to vtkActor types.
class TextureSetterEngine : public RenderEngineVtk {
 public:
  TextureSetterEngine() = default;

  // Reports if the color actor for the geometry with the given `id` has the
  // property texture append by this class's DoRegisterVisual() implementation.
  // This only tests the first actor for the geometry.
  bool GeometryHasColorTexture(GeometryId id,
                               const std::string& texture_name) const {
    vtkActor* actor = RenderEngineVtkTester::GetColorActor(*this, id);
    return actor->GetProperty()->GetTexture(texture_name.c_str()) != nullptr;
  }

  // Applies a texture with the given name to the color actor for the geometry
  // indicated by the given id. This only tests the first actor for the
  // geometry.
  void ApplyColorTextureToGeometry(GeometryId id,
                                   const std::string& texture_name) {
    vtkActor* actor = RenderEngineVtkTester::GetColorActor(*this, id);
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
  const ColorRenderCamera camera(depth_camera_.core(), FLAGS_show_window);
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
    TestColor expected_color;
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
          TestColor dut(image.at(c, r));
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
    TestColor expected_color;
    std::string description;
    std::string target_type;
  };

  // 45-degree vertical field of view.
  const ColorRenderCamera camera(depth_camera_.core(), FLAGS_show_window);
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
  const Rgba kTerrainRgba(kTerrainColor.ToRgba());
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

        const TestColor test_color(image.at(cx, cy));
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
  const ColorRenderCamera camera(depth_camera_.core(), FLAGS_show_window);
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

  const TestColor test_color(image.at(cx, cy));
  const TestColor expected_color = kTerrainColor.ToRgba().scale_rgb(0.75);
  EXPECT_TRUE(IsColorNear(test_color, expected_color))
      << "  test color: " << test_color << "\n"
      << "  expected color: " << expected_color;
}

// Confirms that the environment maps gets parsed and instantiated correctly.
// The texture map is of a box-like room where each surface is a different
// light-emitting color (in high- and low-dynamic range representations).
//
// When the environment map is provided, it illuminates the sphere, whether we
// draw the map in a skybox or not. What we see in the *background* depends on
// whether we request use of the skybox. Either way, the sphere will be the
// same.
//
// The sphere is white and is not illuminated like it would be with a simple
// virtual light. The environment tints the sphere (a function of how much
// light energy is in the environment map). If the camera is looking in the +Wx
// direction, the visible center of the sphere is most reflecting the
// environment color from the -Wy. The colors encoded in the test below reflect
// this.
//
// We want to catch any regression changes that suggest the environment map
// is being registered in the world differently from what is expected. To that
// end, we'll render three different angles, confirming the background and
// sphere illumination values.
//
// Under xvfb, these renderings are *expensive*. So, we'll be very judicious
// in the renderings we do. We'll do one rendering using the high-dynamic range
// map in all three directions with sky box enabled. We'll have one more test
// for each of the following:
//
//    - the skybox property (turning it on and off).
//    - confirming that it survives cloning (do we still see the effects of the
//      environment map).
//    - that low-dynamic range image (we'll assume if it appears as expected
//      from one view, that it's aligned the same as the HDR image).
TEST_F(RenderEngineVtkTest, EnvironmentMap) {
  struct Config {
    std::string description;
    RotationMatrixd R_WC;
    Rgba bg_color;
    Rgba sphere_color;
    std::string map_path;
    bool show_map{true};
    bool render_clone{false};
  };

  const Vector3d clear_rgb = RenderEngineVtkParams().default_clear_color;
  const Rgba clear_color(clear_rgb(0), clear_rgb(1), clear_rgb(2));
  const std::string hdr_path =
      FindResourceOrThrow("drake/geometry/test/env_256_six_color_room.hdr");
  const std::string ldr_path =
      FindResourceOrThrow("drake/geometry/test/env_256_six_color_room.png");

  // 45-degree vertical field of view.
  const ColorRenderCamera camera(depth_camera_.core(), FLAGS_show_window);
  // The camera is three meters removed from the sphere. As we change camera
  // orientation, we'll rotate the camera around the origin at this fixed
  // distance.
  const Vector3d p_WC_C(0, 0, -3);

  ImageRgba8U image(camera.core().intrinsics().width(),
                    camera.core().intrinsics().height());
  // Center pixel -- middle of the sphere.
  const int cx = image.width() / 2;
  const int cy = image.height() / 2;
  // Edge pixel -- so we can see the background.
  const int ex = 10;
  const int ey = 10;

  // Add a white sphere to reflect the environment map's illumination.
  Sphere sphere{0.5};
  PerceptionProperties material;
  material.AddProperty("label", "id", RenderLabel::kDontCare);
  material.AddProperty("phong", "diffuse", Rgba(1, 1, 1));

  // The expected colors are the *observed* colors. Perform the rendering, look
  // at the result, confirm it's what we expect to see, and encode those
  // colors here. We don't have a closed-form solution for predicting the
  // expected colors.
  //
  // Furthermore, the expected background varies between LDR and HDR for a
  // couple of reasons:
  //   1. The HDR image gets tone mapped.
  //   2. The PBR illumination model isn't as simplistic as the phong, so
  //      material colors seldom get reproduced verbatim.
  // This is why it is necessary to empirically define the "expected" colors.
  const std::vector<Config> configs{
      {.description = "Facing +Wz, toward the blue face, magenta behind; HDR",
       .R_WC = RotationMatrixd(),
       .bg_color = Rgba(0, 0, 1),
       .sphere_color = Rgba(0.9882, 0.6353, 0.9098),  // magenta-ish
       .map_path = hdr_path},
      {.description = "Facing blue; testing the skybox",
       .R_WC = RotationMatrixd(),
       .bg_color = Rgba(0, 0, 1),
       .sphere_color = Rgba(0.9882, 0.6353, 0.9098),  // magenta-ish
       .map_path = hdr_path,
       .show_map = false},
      {.description = "Facing blue; testing the clone",
       .R_WC = RotationMatrixd(),
       .bg_color = Rgba(0, 0, 1),
       .sphere_color = Rgba(0.9882, 0.6353, 0.9098),  // magenta-ish
       .map_path = hdr_path,
       .render_clone = true},
      {.description = "Facing +Wy, toward the green face, yellow behind; HDR",
       .R_WC = RotationMatrixd::MakeXRotation(M_PI / 2),
       .bg_color = Rgba(0, 1, 0),
       .sphere_color = Rgba(0.9843, 0.9098, 0.6353),  // yellow-ish
       .map_path = hdr_path},
      {.description = "Facing +Wx, toward the red face, cyan behind; HDR",
       .R_WC = RotationMatrixd::MakeYRotation(M_PI / 2),
       .bg_color = Rgba(1, 0, 0),
       .sphere_color = Rgba(0.5177, 0.9804, 0.9765),  // cyan-ish
       .map_path = hdr_path},
      {.description = "Facing +Wz, toward the blue face, magenta behind; LDR",
       .R_WC = RotationMatrixd(),
       .bg_color = Rgba(0.0588, 0.0588, 0.9255),
       .sphere_color = Rgba(0.7255, 0.4275, 0.6275),  // magenta-ish
       .map_path = ldr_path},
  };

  for (const auto& config : configs) {
    SCOPED_TRACE(config.description);
    const RenderEngineVtkParams params{
        .environment_map = EnvironmentMap{
            .skybox = config.show_map,
            .texture = EquirectangularMap{.path = config.map_path}}};
    RenderEngineVtk renderer(params);

    const RigidTransformd X_WR(config.R_WC, config.R_WC * p_WC_C);
    InitializeRenderer(X_WR, false /* add terrain */, &renderer);

    renderer.RegisterVisual(geometry_id_, sphere, material,
                            RigidTransformd::Identity(),
                            false /* needs update */);

    RenderEngine* renderer_ptr = &renderer;
    std::unique_ptr<RenderEngine> clone{};
    if (config.render_clone) {
      clone = renderer.Clone();
      renderer_ptr = clone.get();
    }
    EXPECT_NO_THROW(renderer_ptr->RenderColorImage(camera, &image));

    // We're using a rather loose pixel tolerance to accommodate vagaries
    // of CI. The value of 20 is required by focal; we can shrink it when we
    // eliminate focal. 10 should be more than enough.
    constexpr int tolerance = 20;

    // Test the center (illumination on the sphere).
    const TestColor center_color(image.at(cx, cy));
    EXPECT_TRUE(IsColorNear(center_color, config.sphere_color, tolerance))
        << "  test color: " << center_color << "\n"
        << "  expected sphere color: " << config.sphere_color;

    // Test the background (we see the right part of the environment map
    // or none at all, if we're not using the sky box).
    const TestColor edge_color(image.at(ex, ey));
    const TestColor bg_color = config.show_map ? config.bg_color : clear_color;
    EXPECT_TRUE(IsColorNear(edge_color, bg_color, tolerance))
        << "  test bg color: " << edge_color << "\n"
        << "  expected bg color: " << bg_color;
  }
}

// RenderEngineVtk promotes all materials to be PBR materials on two conditions:
//
//  1. Any geometry with intrinsic PBR materials is introduced (e.g., a glTF)
//  2. An environment map is introduced.
//
// (1) has been shown in TEST_F(RenderEngineVtkTest, EnvironmentMap). The
// sphere there has a typical phong material and the fact that it gets
// illuminated based on the environment shows PBR promotion.
//
// This test we'll simply confirm that the introduction of a glTF shows an
// illumination change without any other step (indicating material promotion).
TEST_F(RenderEngineVtkTest, PbrMaterialPromotion) {
  auto test_sphere_color = [this](const TestColor expected_color,
                                  RenderEngineVtk* renderer) {
    const ColorRenderCamera camera(depth_camera_.core(), FLAGS_show_window);
    ImageRgba8U image(camera.core().intrinsics().width(),
                      camera.core().intrinsics().height());
    // Center pixel -- middle of the sphere.
    const int cx = image.width() / 2;
    const int cy = image.height() / 2;

    renderer->RenderColorImage(camera, &image);

    const TestColor sampled_color(image.at(cx, cy));
    EXPECT_TRUE(IsColorNear(sampled_color, expected_color))
        << "  rendered color: " << sampled_color << "\n"
        << "  expected color: " << expected_color;
  };

  // Baseline test; sphere only reproduces the phong color at the center.
  // We'll also use this to confirm that defining an EnvironmentMap with a
  // NullTexture has no effect: no exception and it renders like no map was
  // specified at all.
  {
    SCOPED_TRACE("Baseline");
    const Vector3d bg_rgb{kBgColor.r / 255., kBgColor.g / 255.,
                          kBgColor.b / 255.};
    const RenderEngineVtkParams params{.default_clear_color = bg_rgb,
                                       .environment_map = EnvironmentMap()};
    auto renderer = make_unique<RenderEngineVtk>(params);
    InitializeRenderer(X_WC_, /* add_terrain = */ true, renderer.get());
    PopulateSphereTest(renderer.get(), true);
    test_sphere_color(kTextureColor, renderer.get());
  }

  // Add a glTF file; material promoted to PBR no longer matches Phong color.
  {
    SCOPED_TRACE("glTF added");
    Init(X_WC_, true);
    PopulateSphereTest(renderer_.get(), true);

    // Place a glTF mesh far away from the origin; we can't see it but it
    // should still change how things render.
    const Mesh mesh(FindResourceOrThrow(
        "drake/geometry/render/test/meshes/rainbow_box.gltf"));
    PerceptionProperties material;
    material.AddProperty("label", "id", RenderLabel::kDontCare);
    renderer_->RegisterVisual(GeometryId::get_new_id(), mesh, material,
                              RigidTransformd(Vector3d(30, 0, 0)),
                              false /* needs update */);

    // We should still basically be green (because of the green texture), but
    // the saturation and brightness changes in the presence of PBR material.
    const TestColor pbr_texture_color(66, 152, 68, 255);
    test_sphere_color(pbr_texture_color, renderer_.get());
  }
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
  const TestColor ground(kTerrainColor);
  const TestColor curr(curr_pixel);
  const TestColor next(next_pixel);

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
      {"n/a", ref_intrinsics, {clip_n, clip_f}, {}}, FLAGS_show_window};
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
        {"n/a", intrinsics, {clip_n, clip_f}, {}}, FLAGS_show_window};
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
        {"n/a", ref_intrinsics, {n_alt, f_alt}, {}}, FLAGS_show_window};
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
        {"n/a", ref_intrinsics, {n_alt, f_alt}, {}}, FLAGS_show_window};
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

namespace {

// Configure what gets added by AddShapeRows().
struct WholeImageConfig {
  // If true, a textured glTF is used in place of a texture .obj.
  bool gltf_mesh{};

  // If true, a box is used in place of a half space -- necessary for shadows
  // from a directional light. See documentation on
  // RenderEngineVtkParams::cast_shadows for details.
  bool no_half_space{};
};

// Adds rows of Shapes to the given `render_engine`. For each shape type,
// where appropriate, exercises the Shape api and geometry properties to produce
// various variations of how that shape *could* be rendered.
// The rows start at x = 0 and extend in the -x direction.
// If `render_engine` is null, the return value is what the value would be if
// the shapes had been registered.
// @returns the x-value for the shape farthest from the origin (along Wx).
double AddShapeRows(RenderEngineVtk* render_engine,
                    const WholeImageConfig& config = {}) {
  auto register_visual =
      [render_engine](GeometryId id, const Shape& shape,
                      const PerceptionProperties& properties,
                      const std::variant<Vector3d, RigidTransformd>& pose) {
        if (render_engine != nullptr) {
          // Every geometry gets a small rotation around its vertical axis; we
          // want to make sure rotation is properly accounted for.
          const RotationMatrixd R_WG = RotationMatrixd::MakeZRotation(M_PI / 6);
          const RigidTransformd X_WG =
              std::holds_alternative<Vector3d>(pose)
                  ? RigidTransformd(R_WG, std::get<0>(pose))
                  : std::get<1>(pose);
          render_engine->RegisterVisual(id, shape, properties, X_WG,
                                        /* needs_update = */ false);
        }
      };

  int label_value = 0;
  // We want variety among render labels (to confirm that they all matter).
  // Each invocation of minimum_material creates a unique render label value.
  // Changing the invocations will change the output label image.
  auto minimum_material = [&label_value]() {
    PerceptionProperties material;
    // When viewing a label image in something like, gimp, we'd like to be
    // able to distinguish different labels. However, when the 15 bits of value
    // is compressed to 8 bits in the image viewer, values that only differ
    // in the last 8 bits are indistinguishable from each other. So, we'll
    // make sure that the labels differ in the higher bits. Shifting the
    // counter 8 bits would suffice. But we want the values to *visually*
    // differ, so, after the necessary 8 bits, we shift two more bits so that
    // there is a bigger visual distance.
    material.AddProperty("label", "id", RenderLabel(++label_value << 10));
    return material;
  };

  // Each generated diffuse and texture material will pick up a unique label by
  // invoking minimum_material().
  auto diffuse_material = [&minimum_material](const Rgba& rgba) {
    PerceptionProperties material(minimum_material());
    material.AddProperty("phong", "diffuse", rgba);
    return material;
  };

  auto texture_material = [&minimum_material]() {
    PerceptionProperties material(minimum_material());
    material.AddProperty("phong", "diffuse", Rgba(1, 1, 1));
    material.AddProperty(
        "phong", "diffuse_map",
        FindResourceOrThrow("drake/geometry/render/test/meshes/checker.png"));
    return material;
  };

  const double row1 = -0.3;
  const double row2 = 0;
  const double row3 = 0.3;

  double x = 0;
  const Box box(0.1, 0.075, 0.05);
  register_visual(GeometryId::get_new_id(), box,
                  diffuse_material(Rgba(1, 0.25, 0.25)), Vector3d{x, row1, 0});
  register_visual(GeometryId::get_new_id(), box, texture_material(),
                  Vector3d{x, row2, 0});
  PerceptionProperties scaled_texture_material(texture_material());
  scaled_texture_material.AddProperty("phong", "diffuse_scale",
                                      Vector2d{0.5, 0.5});
  register_visual(GeometryId::get_new_id(), box, scaled_texture_material,
                  Vector3d{x, row3, 0});

  x -= 0.15;
  const Capsule capsule(0.05, 0.1);
  register_visual(GeometryId::get_new_id(), capsule,
                  diffuse_material(Rgba(1, 1, 0.25)), Vector3d{x, row1, 0});
  register_visual(GeometryId::get_new_id(), capsule, texture_material(),
                  Vector3d{x, row2, 0});

  // Convex is treated the same as Mesh. We'll put a token convex shape in, but
  // do the rigorous testing on Mesh below.
  x -= 0.15;
  const Convex convex(
      FindResourceOrThrow("drake/geometry/render/test/meshes/box_no_mtl.obj"),
      0.05);
  register_visual(GeometryId::get_new_id(), convex,
                  diffuse_material(Rgba(0.8, 0.25, 0.8)), Vector3d{x, row1, 0});

  x -= 0.15;
  const Cylinder cylinder(0.05, 0.1);
  register_visual(GeometryId::get_new_id(), cylinder,
                  diffuse_material(Rgba(0.25, 1, 0.25)), Vector3d{x, row1, 0});
  register_visual(GeometryId::get_new_id(), cylinder, texture_material(),
                  Vector3d{x, row2, 0});

  x -= 0.15;
  const Ellipsoid ellipsoid(0.05, 0.025, 0.0375);
  register_visual(GeometryId::get_new_id(), ellipsoid,
                  diffuse_material(Rgba(0.25, 1, 1)), Vector3d{x, row1, 0});
  register_visual(GeometryId::get_new_id(), ellipsoid, texture_material(),
                  Vector3d{x, row2, 0});

  // See below for half space.

  x -= 0.15;
  // This mesh has materials (box_checkered.obj or fully_textured_pyramid.gltf).
  // Drake is supposed to ignore any further material specifications and we want
  // to confirm that, at the cost of spewing warnings.
  const Mesh mesh1(
      FindResourceOrThrow(
          config.gltf_mesh
              ? "drake/geometry/render/test/meshes/fully_textured_pyramid.gltf"
              : "drake/geometry/render/test/meshes/box_checkered.obj"),
      0.05);
  register_visual(GeometryId::get_new_id(), mesh1,
                  diffuse_material(Rgba(0.25, 1, 0.25)), Vector3d{x, row1, 0});

  register_visual(GeometryId::get_new_id(), mesh1, minimum_material(),
                  Vector3d{x, row2, 0});

  register_visual(GeometryId::get_new_id(), mesh1, texture_material(),
                  Vector3d{x, row3, 0});

  x -= 0.15;
  const Mesh mesh2(
      FindResourceOrThrow("drake/geometry/render/test/meshes/box_no_mtl.obj"),
      0.05);
  register_visual(GeometryId::get_new_id(), mesh2,
                  diffuse_material(Rgba(0.25, 1, 0.25)), Vector3d{x, row1, 0});

  register_visual(GeometryId::get_new_id(), mesh2, minimum_material(),
                  Vector3d{x, row2, 0});

  register_visual(GeometryId::get_new_id(), mesh2, texture_material(),
                  Vector3d{x, row3, 0});

  x -= 0.15;
  const Sphere sphere(0.05);
  register_visual(GeometryId::get_new_id(), sphere,
                  diffuse_material(Rgba(0.25, 0.25, 1)), Vector3d{x, row1, 0});
  register_visual(GeometryId::get_new_id(), sphere, texture_material(),
                  Vector3d{x, row2, 0});

  // Include a transparent object. We should see through it to another box.
  // With shadows, it should receive but not cast shadows.
  {
    const double mid_x = x * 0.5;
    const Box slab(0.2, 0.2, 0.025);
    register_visual(GeometryId::get_new_id(), slab,
                    diffuse_material(Rgba(0.8, 0.5, 0.5)),
                    RigidTransformd(Vector3d(mid_x + 0.05, row3, 0.1)));
    // Just a little transparent means it won't cast or receive shadows.
    register_visual(GeometryId::get_new_id(), slab,
                    diffuse_material(Rgba(0.5, 0.8, 0.5, 0.5)),
                    RigidTransformd(Vector3d(mid_x + 0.1, row3 + 0.05, 0.2)));
    register_visual(GeometryId::get_new_id(), slab,
                    diffuse_material(Rgba(0.5, 0.5, 0.8)),
                    RigidTransformd(Vector3d(mid_x + 0.15, row3 + 0.1, 0.3)));
  }

  // We've deferred the ground plane until after everything else has been
  // placed so that when we use a box instead of a half space, we can limit its
  // size by centering it on the rows.
  const Rgba ground_color(0.5, 0.25, 0.5);
  if (config.no_half_space) {
    const Box ground(-3 * x, -3 * x, 1.0);
    register_visual(GeometryId::get_new_id(), ground,
                    diffuse_material(ground_color),
                    RigidTransformd(Vector3d{0.5 * x, 0, -0.5 - 0.2}));
  } else {
    // HalfSpace - no textured version; it doesn't support textures.
    const HalfSpace half_space;
    register_visual(GeometryId::get_new_id(), half_space,
                    diffuse_material(ground_color), Vector3d{0, 0, -0.2});
  }

  return x;
}

// Compares the test image against a reference image.
//
// The test image will be written to the test outputs (if defined) with the same
// name as the reference file, but with "_test" appended to the filename. The
// bytes of the image are compared to within the given tolerance.
//
// We want the referenced images to be meaningful to a human as well. So, we
// adopt the following strategy:
//
//   1. color images: No extra action taken.
//   2. label images: To view the 16-bit image in an 8-bit world, we use label
//                    values that truncate nicely (see AddShapeRows()).
//   3. depth images: To view the 32-bit depth image in an 8-bit world, we
//                    color the depth via ColorizeDepthImage. We are not saving
//                    the literal depth values to disk. By implication, it means
//                    we're also only *testing* the color-encoded depth as well.
template <typename ImageType>
void CompareImages(const ImageType& test_image, const std::string& ref_filename,
                   double tolerance, std::string_view log_suffix = {}) {
  // The type we save to disk and compare the bytes of; may not be the same as
  // the input image type.
  using CompareType =
      std::conditional_t<std::is_same_v<ImageType, ImageDepth32F>, ImageRgba8U,
                         ImageType>;
  CompareType compare_image;
  if constexpr (std::is_same_v<ImageType, ImageDepth32F>) {
    // Normalize the depth image so that the resulting image is interpretable
    // by humans.
    ColorizeDepthImage<double> colorizer;
    colorizer.Calc(test_image, &compare_image);
  } else {
    compare_image = test_image;
  }

  const std::filesystem::path ref_path = FindResourceOrThrow(ref_filename);
  if (const char* dir = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR")) {
    const std::filesystem::path out_dir(dir);
    ImageIo{}.Save(compare_image,
                   out_dir / fmt::format("{}{}_test.png",
                                         ref_path.stem().string(), log_suffix));
  }

  CompareType expected_image;
  ASSERT_TRUE(systems::sensors::LoadImage(ref_path, &expected_image));

  // Confirming image equivalence is tricky. We want to be sensitive to changes
  // in code that might produce different images but, at the same time, write a
  // test that will survive the vagaries of rendering in CI. To that end, we
  // employ two thresholds:
  //
  //   - tolerance: allowed per-channel deviation (passed as parameter).
  //   - conformity: the fraction of channel values that must deviate less than
  //                 `tolerance` (hard-coded below).
  //
  // Tolerance is provided by each test, but the conformity is hard-coded to be
  // a high value (99.5%). Essentially, each of the primitives added by
  // AddShapeRows() fills about 0.5% of the final image. If we want to recognize
  // when any of those shapes changes in some significant way, we need to fail
  // if that number of pixels deviates.
  using T = typename CompareType::T;
  ASSERT_EQ(expected_image.size(), compare_image.size());
  Eigen::Map<const VectorX<T>> data_expected(expected_image.at(0, 0),
                                             expected_image.size());
  Eigen::Map<const VectorX<T>> data_actual(compare_image.at(0, 0),
                                           compare_image.size());
  const Eigen::ArrayXd differences = (data_expected.template cast<double>() -
                                      data_actual.template cast<double>())
                                         .array()
                                         .abs();
  const int num_acceptable = (differences <= tolerance).count();
  const double kConformity = 0.995;
  EXPECT_GE(num_acceptable / static_cast<float>(expected_image.size()),
            kConformity);
}

// Given lights measured and expressed in World, re-expresses them in Camera.
vector<LightParameter> TransformLightsToCamera(
    const vector<LightParameter>& lights_W, const RigidTransformd& X_WC) {
  const RigidTransformd X_CW = X_WC.inverse();
  vector<LightParameter> lights_C(lights_W);
  for (LightParameter& light : lights_C) {
    light.position = X_CW * light.position;
    light.direction = X_CW.rotation() * light.direction;
    light.frame = "camera";
  }
  return lights_C;
}

}  // namespace

/* TODO(SeanCurtis-TRI) Figure out how to roll some of the other tests above
 into these single-image renderings, including:

   - glTF models (and their implicit affect of promoting other materials to PBR)
   - Multi-material meshes
   - The light tests.
 */

// Tests multiple rendering features of the render engine by creating one
// "complex" scene and rendering it. It compares the rendered image to a
// reference image. We render all the geometries as anchored, because we're
// not testing the engine's ability to update state, but just testing the
// render features -- textures, materials, lighting, etc.
TEST_F(RenderEngineVtkTest, WholeImageDefaultParams) {
  RenderEngineVtk engine;

  // We'll use the same camera as the default camera for the tests, but shrink
  // the image size so they're not as big in the repository.
  const int w = 480;
  const int h = 360;
  const CameraInfo& source_intrinsics = depth_camera_.core().intrinsics();
  const CameraInfo intrinsics(w, h, source_intrinsics.fov_y());
  const DepthRenderCamera camera(
      {"unused", intrinsics, depth_camera_.core().clipping(),
       depth_camera_.core().sensor_pose_in_camera_body()},
      depth_camera_.depth_range());

  const double last_x = AddShapeRows(&engine);
  // Now make the rendering.
  // The camera is above the Wz = 0 plane, looking generally down and in the
  // +Wy direction (across the rows of shapes).
  const RigidTransformd X_WR(RotationMatrixd::MakeXRotation(-3.2 * M_PI / 4),
                             Vector3d(last_x / 2, -0.75, 1.1));
  engine.UpdateViewpoint(X_WR);

  ImageRgba8U color(w, h);
  ImageDepth32F depth(w, h);
  ImageLabel16I label(w, h);
  this->Render(&engine, &camera, &color, &depth, &label);

  {
    SCOPED_TRACE("Color image");
    CompareImages(
        color, "drake/geometry/render_vtk/test/whole_image_default_color.png",
        /* tolerance = */ 2);
  }
  {
    SCOPED_TRACE("Depth image");
    CompareImages(
        depth, "drake/geometry/render_vtk/test/whole_image_default_depth.png",
        /* tolerance = */ 2);
  }
  {
    SCOPED_TRACE("Label image");
    CompareImages(
        label, "drake/geometry/render_vtk/test/whole_image_default_label.png",
        /* tolerance = */ 0);
  }
}

// Like WholeImageDefaultParams, except we use non-default render engine
// parameters to detect differences.
TEST_F(RenderEngineVtkTest, WholeImageCustomParams) {
  // We need to determine the camera position up front, so we can transform
  // world-frame light definitions into the camera frame. We can determine the
  // extent of added shapes by omitting the render engine and getting the
  // expected result of actually registering geometry.
  const double last_x =
      AddShapeRows(nullptr, {.gltf_mesh = true, .no_half_space = true});
  // The camera is above the Wz plane, on the +Wz side of the x-y plane, looking
  // down and across the x-y plane (looking across the rows of shapes).
  const RigidTransformd X_WR(RotationMatrixd::MakeXRotation(-3.2 * M_PI / 4),
                             Vector3d(last_x / 2, -0.75, 1.1));

  const std::string hdr_path =
      FindResourceOrThrow("drake/geometry/test/env_256_six_color_room.hdr");
  vector<LightParameter> lights_W{{.type = "point",
                                   .color = Rgba(1.0, 0.75, 0.75),
                                   .attenuation_values = {0, 0, 1},
                                   .position = Vector3d(0, 0, 0.3),
                                   .frame = "world",
                                   .intensity = 0.5},
                                  {.type = "spot",
                                   .color = Rgba(0.75, 1.0, 0.75),
                                   .position = Vector3d(0, -0.1, 2),
                                   .frame = "world",
                                   .intensity = 1,
                                   .direction = Vector3d(-0.5, 0, -1),
                                   .cone_angle = 10},
                                  {.type = "directional",
                                   .color = Rgba(1, 1, 1),
                                   .frame = "world",
                                   .intensity = 1,
                                   .direction = Vector3d(0.02, -0.05, -1)}};
  // We should get the same images whether the lights are expressed in the
  // world frame or the camera frame.
  // TODO(SeanCurtis-TRI): When we can edit the lights after instantiation,
  // simplify this so we don't re-instantiate the engine.
  for (const vector<LightParameter>& lights :
       {lights_W, TransformLightsToCamera(lights_W, X_WR)}) {
    const RenderEngineVtkParams params{
        .default_diffuse = Eigen::Vector4d(0.1, 0.2, 0.4, 1.0),
        .default_clear_color = Vector3d(0.25, 0.25, 0.25),
        .lights = lights,
        .environment_map = {EnvironmentMap{
            // Note: The ground plane covers the full background, we won't be
            // able to see the skybox.
            .texture = EquirectangularMap{.path = hdr_path}}},
        .exposure = 0.75,
        .cast_shadows = true,
        .shadow_map_size = 1024};
    RenderEngineVtk engine(params);

    // We'll use the same camera as the default camera for the tests, but shrink
    // the image size so they're not as big in the repository.
    const int w = 480;
    const int h = 360;
    const CameraInfo& source_intrinsics = depth_camera_.core().intrinsics();
    const CameraInfo intrinsics(w, h, source_intrinsics.fov_y());
    const DepthRenderCamera camera(
        {"unused", intrinsics, depth_camera_.core().clipping(),
         depth_camera_.core().sensor_pose_in_camera_body()},
        depth_camera_.depth_range());

    // The value we used to define the camera pose better match.
    DRAKE_DEMAND(AddShapeRows(&engine, {.gltf_mesh = true,
                                        .no_half_space = true}) == last_x);

    // Now make the rendering.
    engine.UpdateViewpoint(X_WR);

    ImageRgba8U color(w, h);
    ImageDepth32F depth(w, h);
    ImageLabel16I label(w, h);
    this->Render(&engine, &camera, &color, &depth, &label);

    {
      SCOPED_TRACE(fmt::format("Color image - {}", lights[0].frame));
      // We only need this for color, as neither depth nor label images are
      // affected by shadows.
      const std::string log_suffix = fmt::format("_{}", lights[0].frame);
      CompareImages(
          color, "drake/geometry/render_vtk/test/whole_image_custom_color.png",
          /* tolerance = */ 2, log_suffix);
    }
    {
      SCOPED_TRACE("Depth image");
      CompareImages(
          depth, "drake/geometry/render_vtk/test/whole_image_custom_depth.png",
          /* tolerance = */ 2);
    }
    {
      SCOPED_TRACE("Label image");
      CompareImages(
          label, "drake/geometry/render_vtk/test/whole_image_custom_label.png",
          /* tolerance = */ 0);
    }
  }
}

// To work around a bug in VTK, when shadows are enabled, we have to render into
// a square window, and then crop the image back out. The WholeImageCustomParams
// implicitly handles non-square images with a *horizontal* aspect ratio. This
// test repeats that test, but with a vertical aspect ratio.
//
// Essentially, we create a camera with a taller window (double the height), but
// the same width and focal lengths. This should have the effect of embedding
// the expected reference image in the vertical center of the resulting image.
//
// When we no longer require the render-to-square-window hack, we can nuke this
// test.
TEST_F(RenderEngineVtkTest, WholeImageVerticalAspectRatio) {
  // From here we basically have the setup boilerplate. Differences are
  // explicitly called out.

  // These engine parameters must match those in WholeImageCustomParams.
  const std::string hdr_path =
      FindResourceOrThrow("drake/geometry/test/env_256_six_color_room.hdr");
  const RenderEngineVtkParams params{
      .default_diffuse = Eigen::Vector4d(0.1, 0.2, 0.4, 1.0),
      .default_clear_color = Vector3d(0.25, 0.25, 0.25),
      .lights = {{.type = "point",
                  .color = Rgba(1.0, 0.75, 0.75),
                  .attenuation_values = {0, 0, 1},
                  .position = Vector3d(0, 0, 0.3),
                  .frame = "world",
                  .intensity = 0.5},
                 {.type = "spot",
                  .color = Rgba(0.75, 1.0, 0.75),
                  .position = Vector3d(0, -0.1, 2),
                  .frame = "world",
                  .intensity = 1,
                  .direction = Vector3d(-0.5, 0, -1),
                  .cone_angle = 10},
                 {.type = "directional",
                  .color = Rgba(1, 1, 1),
                  .frame = "world",
                  .intensity = 1,
                  .direction = Vector3d(0.02, -0.05, -1)}},
      .environment_map = {EnvironmentMap{
          // Note: The ground plane covers the full background, we won't be
          // able to see the skybox.
          .texture = EquirectangularMap{.path = hdr_path}}},
      .exposure = 0.75,
      .cast_shadows = true,
      .shadow_map_size = 1024};
  RenderEngineVtk engine(params);

  // These are the intrinsics used by the custom test with horizontal aspect
  // ratio. We'll use it to make sure we use the same focal lengths.
  const CameraInfo ref_intrinsics(480, 360,
                                  depth_camera_.core().intrinsics().fov_y());

  // The vertical-aspect-ratio version for this test.
  const int w = ref_intrinsics.width();
  const int h = ref_intrinsics.height() * 2;
  DRAKE_DEMAND(h > w);  // It's really vertical, right?

  // Note {w|h} * 0.5 - 0.5 is the center calculation otherwise embedded in
  // CameraInfo; we have to duplicate it here so we can dictate focal lengths.
  const CameraInfo intrinsics(w, h, ref_intrinsics.focal_x(),
                              ref_intrinsics.focal_y(), w * 0.5 - 0.5,
                              h * 0.5 - 0.5);
  const DepthRenderCamera camera(
      {"unused", intrinsics, depth_camera_.core().clipping(),
       depth_camera_.core().sensor_pose_in_camera_body()},
      depth_camera_.depth_range());

  // More boilerplate in common with the whole image custom parameters test.
  const double last_x =
      AddShapeRows(&engine, {.gltf_mesh = true, .no_half_space = true});
  // Now make the rendering.
  // The camera is above the Wz = 0 plane, looking generally down and in the
  // +Wy direction (across the rows of shapes).
  const RigidTransformd X_WR(RotationMatrixd::MakeXRotation(-3.2 * M_PI / 4),
                             Vector3d(last_x / 2, -0.75, 1.1));
  engine.UpdateViewpoint(X_WR);

  // Render targets have to match declared size.
  ImageRgba8U tall_color(w, h);
  ImageDepth32F tall_depth(w, h);
  ImageLabel16I tall_label(w, h);
  this->Render(&engine, &camera, &tall_color, &tall_depth, &tall_label);

  // Now clip down to reference size. We only need to clip the color image;
  // label and depth images don't get the render-as-square treatment because
  // they don't get shadows.
  auto clip_image = [](const auto& tall_image, auto* image) {
    DRAKE_DEMAND(tall_image.width() == image->width());
    int source_row = (tall_image.height() - image->height()) / 2;
    const auto* source_ptr = tall_image.at(0, source_row);
    auto* dest_ptr = image->at(0, 0);
    const int kPixelSize = std::remove_pointer_t<decltype(image)>::kPixelSize;
    memcpy(dest_ptr, source_ptr, image->width() * image->height() * kPixelSize);
  };
  ImageRgba8U color(w, h / 2);
  clip_image(tall_color, &color);

  CompareImages(color,
                "drake/geometry/render_vtk/test/whole_image_custom_color.png",
                /* tolerance = */ 2, "_vertical_ar");
}

}  // namespace
}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
