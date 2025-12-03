#include "drake/geometry/render_gl/internal_render_engine_gl.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <source_location>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkImageData.h>  // vtkCommonDataModel
#include <vtkNew.h>        // vtkCommonCore
#include <vtkPNGReader.h>  // vtkIOImage

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/read_gltf_to_memory.h"
#include "drake/geometry/render/render_label.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/image_io.h"
#include "drake/systems/sensors/test_utilities/image_compare.h"

DEFINE_bool(show_window, false, "Display render windows locally for debugging");

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {

namespace fs = std::filesystem;

using render::ColorRenderCamera;
using render::DepthRenderCamera;
using render::LightParameter;
using render::RenderCameraCore;
using render::RenderEngine;
using render::RenderLabel;

// Friend class that gives the tests access to a RenderEngineGl's OpenGlContext.
class RenderEngineGlTester {
 public:
  using Prop = RenderEngineGl::Prop;

  /* Constructs a tester on the given engine. The tester keeps a reference to
   the given `engine`; the engine must stay alive at least as long as the
   tester.  */
  explicit RenderEngineGlTester(const RenderEngineGl* engine)
      : engine_(DRAKE_DEREF(engine)) {}

  const internal::OpenGlContext& opengl_context() const {
    return *engine_.opengl_context_;
  }

  const Prop& GetVisual(GeometryId id) {
    DRAKE_DEMAND(engine_.visuals_.contains(id));
    return engine_.visuals_.at(id);
  }

  const TextureLibrary& texture_library() const {
    return *engine_.texture_library_;
  }

  const std::vector<internal::OpenGlGeometry>& opengl_geometries() const {
    return engine_.geometries_;
  }

 private:
  const RenderEngineGl& engine_;
};

namespace {

using Eigen::AngleAxisd;
using Eigen::DiagonalMatrix;
using Eigen::Matrix4d;
using Eigen::Translation3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using math::RigidTransformd;
using math::RotationMatrixd;
using std::make_unique;
using std::unique_ptr;
using std::unordered_map;
using systems::sensors::CameraInfo;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageIo;
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
// We need a color that we can see the effects of illumination on.
const Rgba kTerrainColor{0.5, 0.5, 0.6, 1};
// box.png contains a single pixel with the color (4, 241, 33). If the image
// changes, the expected color would likewise have to change.
const Rgba kTextureColor{4 / 255.0, 241 / 255.0, 33 / 255.0, 1.0};

// Provide a default visual color for this tests -- it is intended to be
// different from the default color of the OpenGL render engine.
const Rgba kDefaultVisualColor{229 / 255.0, 229 / 255.0, 229 / 255.0, 1.0};
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
// unsigned bytes, as four separate ints, and from a normalized color. It's
// nice to articulate tests without having to worry about those details.
struct RgbaColor {
  explicit RgbaColor(const uint8_t* p) : r(p[0]), g(p[1]), b(p[2]), a(p[3]) {}
  RgbaColor(int r_in, int g_in, int b_in, int a_in)
      : r(r_in), g(g_in), b(b_in), a(a_in) {}
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

  int r{};
  int g{};
  int b{};
  int a{};
};

std::ostream& operator<<(std::ostream& out, const RgbaColor& c) {
  out << "(" << c.r << ", " << c.g << ", " << c.b << ", " << c.a << ")";
  return out;
}

// Tests color within tolerance.
bool IsColorNear(const RgbaColor& expected, const RgbaColor& tested,
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
              {0, 0, kDefaultDistance}) {
    // Tests of glTF support work by creating a family of related files. We'll
    // work in this directory.
    temp_dir_ = temp_directory();
  }

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
    const ColorRenderCamera color_camera(depth_camera.core(),
                                         FLAGS_show_window);
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
  void VerifyOutliers(const DepthRenderCamera& camera,
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

  void SetUp() override { ResetExpectations(); }

  // Tests that don't instantiate their own renderers should invoke this.
  void Init(const RigidTransformd& X_WR, bool add_terrain = false) {
    const RenderEngineGlParams params{.default_clear_color = kBgColor};
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
    sphere_id_ = GeometryId::get_new_id();
    renderer->RegisterVisual(sphere_id_, sphere, simple_material(use_texture),
                             RigidTransformd::Identity(),
                             true /* needs update */);
    RigidTransformd X_WV{Vector3d{0, 0, r}};
    X_WV_.clear();
    X_WV_.insert({sphere_id_, X_WV});
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

  // Performs the work to test the rendering with a shape centered in the
  // image. To pass, the renderer will have to have been populated with a
  // compliant shape and camera configuration (e.g., PopulateSphereTest()).
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

    VerifyCenterShapeTest(cam, color, depth, label);
  }

  void VerifyCenterShapeTest(const DepthRenderCamera& camera,
                             const ImageRgba8U& color,
                             const ImageDepth32F& depth,
                             const ImageLabel16I& label) const {
    VerifyOutliers(camera, &color, &depth, &label);

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

  // Copies some supporting files for the glTF tests into this test's temp
  // directory.
  void SetupGltfTest() {
    // Note: we're copying _all_ of the files associated with
    // fully_textured_pyramid.gltf even though RenderEngineGl only uses a small
    // fraction. tinygltf will attempt to load every named textures, so we need
    // to make sure they're available to avoid warnings.
    for (const char* resource :
         {"fully_textured_pyramid.bin", "fully_textured_pyramid_base_color.png",
          "fully_textured_pyramid_emissive.png",
          "fully_textured_pyramid_normal.png", "fully_textured_pyramid_omr.png",
          "fully_textured_pyramid_emissive.ktx2",
          "fully_textured_pyramid_normal.ktx2",
          "fully_textured_pyramid_omr.ktx2",
          "fully_textured_pyramid_base_color.ktx2"}) {
      const fs::path source_path = FindResourceOrThrow(
          fmt::format("drake/geometry/render/test/meshes/{}", resource));
      fs::copy_file(source_path, temp_dir_ / resource);
    }
  }

  // Returns the path to the fully textured pyramid gltf file.
  static fs::path gltf_pyramid_path() {
    static never_destroyed<fs::path> path(FindResourceOrThrow(
        "drake/geometry/render/test/meshes/fully_textured_pyramid.gltf"));
    return path.access();
  }

  void InitAndRegisterMesh(const fs::path& file_path) {
    Init(RigidTransformd::Identity());
    PerceptionProperties material;
    material.AddProperty("label", "id", RenderLabel(1));
    const GeometryId id = GeometryId::get_new_id();
    renderer_->RegisterVisual(id, Mesh(file_path.string()), material,
                              RigidTransformd::Identity(),
                              false /* needs update */);
  }

  // Renders the built-in renderer with specific camera and rendering settings.
  // Compares the resultant image with a reference image.
  // @pre The renderer has been properly initialized.
  void RenderAndCompareAgainstRef(std::string_view image_file_name,
                                  RenderEngine* renderer_in = nullptr) {
    RenderEngine* renderer =
        renderer_in == nullptr ? renderer_.get() : renderer_in;
    const RotationMatrixd R_WC(math::RollPitchYawd(-M_PI / 2.5, 0, M_PI / 4));
    const RigidTransformd X_WC(
        R_WC, R_WC * Vector3d(0, 0, -6) + Vector3d(0, 0, -0.15));
    renderer->UpdateViewpoint(X_WC);

    ImageRgba8U image(64, 64);
    const ColorRenderCamera camera(
        {"unused", {64, 64, kFovY / 2}, {0.01, 10}, {}}, FLAGS_show_window);
    renderer->RenderColorImage(camera, &image);

    if (const char* dir = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR")) {
      const fs::path out_dir(dir);
      systems::sensors::ImageIo{}.Save(
          image, out_dir / fmt::format("{}.png", image_file_name));
    }

    ImageRgba8U expected_image;
    const std::string ref_filename = FindResourceOrThrow(
        "drake/geometry/render_gl/test/gl_fully_textured_pyramid_rendered.png");
    ASSERT_TRUE(systems::sensors::LoadImage(ref_filename, &expected_image));
    // We're testing to see if the images are *mostly* equal. This accounts
    // for the differences in CI's rendering technology from a local GPU. The
    // images are deemed equivalent if 99% of the channel values are within 2
    // of the reference color.
    ASSERT_EQ(expected_image.size(), image.size());
    Eigen::Map<VectorX<uint8_t>> data_expected(expected_image.at(0, 0),
                                               expected_image.size());
    Eigen::Map<VectorX<uint8_t>> data2(image.at(0, 0), image.size());
    const auto differences =
        (data_expected.cast<float>() - data2.cast<float>()).array().abs();
    const int num_acceptable = (differences <= 2).count();
    EXPECT_GE(num_acceptable / static_cast<float>(expected_image.size()), 0.99);
  }

  RgbaColor expected_color_{kDefaultVisualColor};
  RgbaColor expected_outlier_color_{kTerrainColor};
  float expected_outlier_depth_{kDefaultDistance};
  float expected_object_depth_{2.f};
  RenderLabel expected_label_;
  RenderLabel expected_outlier_label_{RenderLabel::kDontCare};
  RgbaColor default_color_{kDefaultVisualColor};

  // We store a reference depth camera; we can always derive a color camera
  // from it; they have the same intrinsics and we grab the global
  // FLAGS_show_window.
  const DepthRenderCamera depth_camera_{
      {"unused", {kWidth, kHeight, kFovY}, {kClipNear, kClipFar}, {}},
      {kZNear, kZFar}};

  ImageRgba8U color_;
  ImageDepth32F depth_;
  ImageLabel16I label_;
  RigidTransformd X_WR_;
  GeometryId sphere_id_;

  // The pose of the sphere created in PopulateSphereTest().
  unordered_map<GeometryId, RigidTransformd> X_WV_;

  unique_ptr<RenderEngineGl> renderer_;

  fs::path temp_dir_;
};

TEST_F(RenderEngineGlTest, ParameterMatching) {
  auto make_yaml = [](const RenderEngineGlParams& params) {
    return yaml::SaveYamlString(params, "RenderEngineGlParams");
  };
  const RenderEngineGlParams params1{
      .lights = {LightParameter{.type = "spot"}}};
  const RenderEngineGlParams params2;

  const RenderEngineGl engine(params1);
  const std::string from_engine = engine.GetParameterYaml();

  EXPECT_EQ(from_engine, make_yaml(params1));
  EXPECT_NE(from_engine, make_yaml(params2));
}

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
    const RenderEngineGlParams params{.default_clear_color = bg};
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

  const ColorRenderCamera camera(depth_camera_.core(), FLAGS_show_window);
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
        expected_color_ = RgbaColor(135, 116, 15, 255);
        // Quick proof that we're testing for a different color -- we're drawing
        // the red channel from our expected color.
        ASSERT_NE(kTextureColor.r(), expected_color_.r);
      } else {
        // Otherwise the expected is simply the texture color of box.png.
        expected_color_ =
            RgbaColor(use_texture ? kTextureColor : default_color_);
      }

      SCOPED_TRACE(fmt::format("Box test - {} - scale {}",
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
  RenderEngineGlParams params;
  RenderEngineGl renderer{params};
  // TODO(20206): This test depends on the terrain having a *smaller* geometry
  // id than the sphere, so that it gets rendered first and is visible through
  // the sphere. Once transparency is handled better, we should confirm we get
  // the right result, regardless of registration order.
  InitializeRenderer(X_WR_, true /* add terrain */, &renderer);
  const int int_alpha = 128;
  // Sets the color of the sphere that will be created in PopulateSphereTest.
  default_color_ = Rgba(kDefaultVisualColor.r(), kDefaultVisualColor.g(),
                        kDefaultVisualColor.b(), int_alpha / 255.0);
  PopulateSphereTest(&renderer);
  const ColorRenderCamera camera(depth_camera_.core(), FLAGS_show_window);
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

// Performs the shape-centered-in-the-image test with a deformable mesh. In
// particular, we register a deformable geometry with a single mesh (with or
// without texture) and update the vertex positions and normals with some
// curated values. We then render color, depth, and label images to verify they
// match our expectations at certain pixel locations. Though this doesn't
// explicitly confirm the vertex positions and normals of all vertices are
// correctly updated, it proves some updates happened and provides strong
// indications that the updates are as expected. Note that this only tests a
// deformable geometry with a single render mesh, and we use the success of that
// test to indicate vertices are correctly updated for all meshes.
TEST_F(RenderEngineGlTest, DeformableTest) {
  for (const bool use_texture : {false, true}) {
    Init(X_WR_, true);
    ResetExpectations();

    // N.B. box_no_mtl.obj doesn't exist in the source tree and is generated
    // from box.obj by stripping out material data in the build system.
    const fs::path filename =
        use_texture
            ? FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj")
            : FindResourceOrThrow(
                  "drake/geometry/render/test/meshes/box_no_mtl.obj");

    RenderLabel deformable_label(847);
    expected_label_ = deformable_label;
    PerceptionProperties material = simple_material(use_texture);
    // This is a dummy placeholder to allow invoking LoadRenderMeshesFromObj(),
    // the actual diffuse color either comes from the mtl file or the
    // perception properties.
    Rgba unused_diffuse_color(1, 1, 1, 1);
    std::vector<geometry::internal::RenderMesh> render_meshes =
        geometry::internal::LoadRenderMeshesFromObj(filename, material,
                                                    unused_diffuse_color);
    ASSERT_EQ(render_meshes.size(), 1);

    const GeometryId id = GeometryId::get_new_id();
    renderer_->RegisterDeformableVisual(id, render_meshes, material);

    expected_color_ = use_texture ? RgbaColor(kTextureColor) : default_color_;

    SCOPED_TRACE(
        fmt::format("Deformable test -- has texture: {}", use_texture));

    PerformCenterShapeTest(renderer_.get());

    const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>&
        initial_q_WG = render_meshes[0].positions;
    const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>&
        initial_nhat_W = render_meshes[0].normals;
    // Helper lambda to translate all vertex positions by the same vector.
    auto translate_all_vertices = [&initial_q_WG](const Vector3d& t_W) {
      auto result = initial_q_WG;
      for (int i = 0; i < result.rows(); ++i) {
        result.row(i) += t_W;
      }
      return result;
    };
    // Helper lambda to reshape an Nx3 matrix to a flat vector with 3N entries.
    auto flatten = [](const Eigen::Matrix<double, Eigen::Dynamic, 3,
                                          Eigen::RowMajor>& input) {
      return VectorXd(Eigen::Map<const VectorXd>(input.data(), input.size()));
    };

    // The box has half edge length 1.0 and has its center and the center of the
    // image. Assuming infinite resolution, when the box is translated by (t_x,
    // 0, 0), we expect the center of the image to be part of the box if t_x is
    // in the interval (-1, 1) and part of the terrain if t_x < -1 or if
    // t_x > 1. With finite resolution, the boundary is somewhat "blurred".

    // Pixel at center renders box.
    renderer_->UpdateDeformableConfigurations(
        id,
        std::vector<VectorXd>{
            flatten(translate_all_vertices(Vector3d(0.99, 0, 0)))},
        std::vector<VectorXd>{flatten(initial_nhat_W)});
    PerformCenterShapeTest(renderer_.get());

    // Pixel at center renders the terrain.
    expected_color_ = expected_outlier_color_;
    expected_object_depth_ = expected_outlier_depth_;
    expected_label_ = expected_outlier_label_;
    renderer_->UpdateDeformableConfigurations(
        id,
        std::vector<VectorXd>{
            flatten(translate_all_vertices(Vector3d(1.01, 0, 0.0)))},
        std::vector<VectorXd>{flatten(initial_nhat_W)});
    PerformCenterShapeTest(renderer_.get());

    // Test normals are updated by making all vertex normals point along the
    // direction of (1, 0, 1) in the world frame. As a result, angle between the
    // normal and the light direction is 45 degrees.
    auto new_nhat_W = initial_nhat_W;
    for (int r = 0; r < new_nhat_W.rows(); ++r) {
      new_nhat_W.row(r) = Vector3d(1, 0, 1).normalized();
    }

    // With the prescribed normals, we expect to see rgb values scaled by
    // cos(π/4). We also expect the object depth to increase by 0.5 as we
    // translate all vertices in the -z direction by 0.5.
    ResetExpectations();
    RgbaColor original_color =
        use_texture ? RgbaColor(kTextureColor) : default_color_;
    const Vector3d original_rgb(original_color.r, original_color.g,
                                original_color.b);
    const Vector3d expected_rgb = original_rgb * std::cos(M_PI / 4.0);
    expected_color_ = RgbaColor(static_cast<int>(expected_rgb[0]),
                                static_cast<int>(expected_rgb[1]),
                                static_cast<int>(expected_rgb[2]), 255);
    expected_label_ = deformable_label;
    expected_object_depth_ += 0.5;

    renderer_->UpdateDeformableConfigurations(
        id,
        std::vector<VectorXd>{
            flatten(translate_all_vertices(Vector3d(0, 0, -0.5)))},
        std::vector<VectorXd>{flatten(new_nhat_W)});
    PerformCenterShapeTest(renderer_.get());

    // Now we remove the geometry, and the center pixel should again render the
    // terrain.
    renderer_->RemoveGeometry(id);
    expected_color_ = expected_outlier_color_;
    expected_object_depth_ = expected_outlier_depth_;
    expected_label_ = expected_outlier_label_;
    PerformCenterShapeTest(renderer_.get());

    // Confirm that we can still add the geometry back.
    renderer_->RegisterDeformableVisual(id, render_meshes, material);
  }
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
  VerifyOutliers(depth_camera_);

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
// box). The textured box will be one that is textured via its mtl library. We
// use it to confirm that the render engine is properly _invoking_ the obj
// material handling; the _correctness_ of the material handling is tested in
// render_mesh_test.cc. For the non-textured, we make sure we use a mesh without
// material file or matching foo.png to preclude it being textured.
TEST_F(RenderEngineGlTest, MeshTest) {
  for (const bool use_texture : {false, true}) {
    Init(X_WR_, true);

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
    PerceptionProperties material = simple_material();
    const GeometryId id = GeometryId::get_new_id();
    renderer_->RegisterVisual(id, mesh, material, RigidTransformd::Identity(),
                              true /* needs update */);
    renderer_->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
        {id, RigidTransformd::Identity()}});

    expected_color_ = use_texture ? RgbaColor(kTextureColor) : default_color_;
    SCOPED_TRACE("Mesh test");
    PerformCenterShapeTest(renderer_.get());

    // For the textured mesh, we also confirm that the texture survives cloning.
    if (use_texture) {
      SCOPED_TRACE("Clone textured mesh test");
      unique_ptr<RenderEngine> clone = renderer_->Clone();
      RenderEngineGl* gl_engine = dynamic_cast<RenderEngineGl*>(clone.get());
      EXPECT_NE(gl_engine, nullptr);
      PerformCenterShapeTest(gl_engine);
    }
  }
}

// Confirm that non-uniform scale is correctly applied. We'll create
// two renderings: one with a reference mesh and one with the mesh pre-scaled
// (applying the inverse scale to the Shape). The two images should end up
// identical.
TEST_F(RenderEngineGlTest, NonUniformScale) {
  const ColorRenderCamera camera(depth_camera_.core(), FLAGS_show_window);
  const int w = camera.core().intrinsics().width();
  const int h = camera.core().intrinsics().height();

  const auto convex_id = GeometryId::get_new_id();
  const auto mesh_id = GeometryId::get_new_id();
  PerceptionProperties material;
  material.AddProperty("label", "id", RenderLabel::kDontCare);

  // The camera is above the Wz = 0 plane, looking generally down and in the
  // +Wy direction.
  const RigidTransformd X_WC(RotationMatrixd::MakeXRotation(-3.2 * M_PI / 4),
                             Vector3d(0, -3, 4.4));

  for (const auto& extension : {"obj", "gltf"}) {
    SCOPED_TRACE(fmt::format("Extension: .{}", extension));
    RenderEngineGl ref_engine;
    RenderEngineGl scale_engine;

    const fs::path unit_mesh = FindResourceOrThrow(fmt::format(
        "drake/geometry/test/rotated_cube_unit_scale.{}", extension));
    const fs::path scale_mesh = FindResourceOrThrow(
        fmt::format("drake/geometry/test/rotated_cube_squished.{}", extension));

    const Vector3d unit_scale(1, 1, 1);
    ref_engine.RegisterVisual(mesh_id, Mesh(unit_mesh, unit_scale), material,
                              RigidTransformd(Vector3d(-1.5, 0, 0)),
                              /* needs_update =*/false);
    ref_engine.RegisterVisual(convex_id, Convex(unit_mesh, unit_scale),
                              material, RigidTransformd(Vector3d(1.5, 0, 0)),
                              /* needs_update =*/false);

    // This should be the scale factor documented in rotated_cube_squished.obj
    const Vector3d stretch(2, 4, 8);
    scale_engine.RegisterVisual(mesh_id, Mesh(scale_mesh, stretch), material,
                                RigidTransformd(Vector3d(-1.5, 0, 0)),
                                /* needs_update =*/false);
    scale_engine.RegisterVisual(convex_id, Convex(scale_mesh, stretch),
                                material, RigidTransformd(Vector3d(1.5, 0, 0)),
                                /* needs_update =*/false);

    ref_engine.UpdateViewpoint(X_WC);
    scale_engine.UpdateViewpoint(X_WC);

    ImageRgba8U ref_color(w, h);
    ImageRgba8U scale_color(w, h);
    EXPECT_NO_THROW(ref_engine.RenderColorImage(camera, &ref_color));
    EXPECT_NO_THROW(scale_engine.RenderColorImage(camera, &scale_color));

    if (const char* dir = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR")) {
      const std::string stem = fmt::format(
          "line_{:0>4}_{}", std::source_location::current().line(), extension);
      const fs::path out_dir(dir);
      ImageIo{}.Save(ref_color,
                     out_dir / fmt::format("{}_ref_color.png", stem));
      ImageIo{}.Save(scale_color,
                     out_dir / fmt::format("{}_scale_color.png", stem));
    }

    EXPECT_EQ(ref_color, scale_color);
  }
}

// Repeats various mesh-based tests, but this time the meshes are loaded from
// memory. We render the scene twice: once with the one mesh and once with the
// other to confirm they are rendered the same.
TEST_F(RenderEngineGlTest, InMemoryMesh) {
  // Pose the camera so we can see three sides of the cubes.
  const RotationMatrixd R_WR(math::RollPitchYawd(-0.75 * M_PI, 0, M_PI_4));
  const RigidTransformd X_WR(R_WR,
                             R_WR * -Vector3d(0, 0, 1.5 * kDefaultDistance));
  Init(X_WR, true);

  const GeometryId id = GeometryId::get_new_id();
  auto do_test = [this, id](std::string_view file_prefix, const Mesh& file_mesh,
                            const Mesh& memory_mesh) {
    renderer_->RemoveGeometry(id);
    renderer_->RegisterVisual(id, file_mesh, PerceptionProperties{},
                              RigidTransformd::Identity(), false);
    ImageRgba8U file_image(kWidth, kHeight);
    Render(nullptr, nullptr, &file_image, nullptr, nullptr);

    renderer_->RemoveGeometry(id);
    renderer_->RegisterVisual(id, memory_mesh, PerceptionProperties{},
                              RigidTransformd::Identity(), false);
    ImageRgba8U memory_image(kWidth, kHeight);
    Render(nullptr, nullptr, &memory_image, nullptr, nullptr);

    if (const char* dir = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR")) {
      const fs::path out_dir(dir);
      ImageIo{}.Save(file_image,
                     out_dir / fmt::format("{}_file.png", file_prefix));
      ImageIo{}.Save(memory_image,
                     out_dir / fmt::format("{}_memory.png", file_prefix));
    }

    EXPECT_TRUE(file_image == memory_image) << fmt::format(
        "The glTF file loaded from disk didn't match that loaded from memory. "
        "Check the bazel-testlogs for the saved images with the prefix '{}'.",
        file_prefix);
  };

  // cube1.gltf has all internal data; this confirms that data uris are
  // preserved.
  {
    const fs::path path =
        FindResourceOrThrow("drake/geometry/render/test/meshes/cube1.gltf");
    InMemoryMesh mesh_data = ReadGltfToMemory(path);
    do_test("embedded_gltf", Mesh(path.string()), Mesh(std::move(mesh_data)));
  }

  // cube2.gltf uses all external files; confirming that file uris work.
  {
    const fs::path path =
        FindResourceOrThrow("drake/geometry/render/test/meshes/cube2.gltf");
    InMemoryMesh mesh_data = ReadGltfToMemory(path);
    do_test("file_uri_gltf", Mesh(path.string()), Mesh(std::move(mesh_data)));
  }

  // rainbow_box.obj has some faces colored by texture, some by material. The
  // rendering includes faces of both types so we can tell if the right
  // materials and textures are getting loaded in the right way.
  {
    const fs::path obj_path = FindResourceOrThrow(
        "drake/geometry/render/test/meshes/rainbow_box.obj");
    const fs::path mtl_path = FindResourceOrThrow(
        "drake/geometry/render/test/meshes/rainbow_box.mtl");
    const fs::path png_path = FindResourceOrThrow(
        "drake/geometry/render/test/meshes/rainbow_stripes.png");
    do_test("textured_obj", Mesh(obj_path.string()),
            Mesh(InMemoryMesh{
                MemoryFile::Make(obj_path),
                {{"rainbow_box.mtl", MemoryFile::Make(mtl_path)},
                 {"rainbow_stripes.png", MemoryFile::Make(png_path)}}}));
  }
}

// A note on testing the glTF support.
//
// These tests are *not* exhaustive. These cover major features and general
// regression tests. Some aspects have actively been left untested. If these
// cause problems in the future, more tests can be added. Such untested features
// include:
//
//  - Support for different numerical types (for both indices and vertex
//    attributes).
//  - Various kinds of primitives (triangle strips, points, lines, etc).
//  - The requirement that all vertex attribute data for a single primitive be
//    contained in a single buffer. The probability of this not being the case
//    is quite small and if Drake encounters such a case, it should throw.
//  - Correct handling of normals with anisotropic scale factors (computation of
//    N_WN). We'll wait to see if people complain about the appearance. But as
//    it is unlikely this will happen and it is difficult to articulate a unit
//    test to verify the correctness, we'll defer it.
//  - There are a number of throwing conditions. Most of them should be
//    considered adversarially bad glTF files. These are conditions in which
//    the glTF is syntactically "legal" but either meaningless or unsupported
//    by Drake. (For this latter case, such cases are deemed to be unlikely.)
//    For example, an index that is larger than the collection it indexes into.
//    Such a glTF is syntactically valid, but meaningless. We're not going to
//    put those throwing conditions under test.

using GltfTweaker = std::function<void(nlohmann::json*)>;

// Loads the json of the input glTF file, applies the `tweaker` operator to it
// and writes it to the output glTF file.
void TweakGltf(const fs::path& gltf_in, const fs::path& gltf_out,
               GltfTweaker tweaker) {
  std::ifstream in(gltf_in);
  DRAKE_DEMAND(in.good());
  nlohmann::json json = nlohmann::json::parse(in);
  tweaker(&json);
  std::ofstream out(gltf_out);
  DRAKE_DEMAND(out.good());
  out << std::setw(2) << json << "\n";
  // TODO(SeanCurtis-TRI): Consider also dumping the modified file to the
  // test output directory.
}

#define DoCompareTest(name)            \
  {                                    \
    SCOPED_TRACE(#name);               \
    InitAndRegisterMesh(name);         \
    RenderAndCompareAgainstRef(#name); \
  }

// Errors detected by tinygltf are forwarded.
TEST_F(RenderEngineGlTest, GltfTinygltfErrors) {
  SetupGltfTest();

  // Malformed gltf file - removed a required field. This error is caught by
  // tinygltf and represents all errors that tinygltf catches (including bad
  // json and the like).
  const fs::path malformed_gltf = temp_dir_ / "malformed_gltf.gltf";
  TweakGltf(gltf_pyramid_path(), malformed_gltf, [](nlohmann::json* model_ptr) {
    nlohmann::json& model = *model_ptr;
    model["accessors"][0].erase("count");
  });
  DRAKE_EXPECT_THROWS_MESSAGE(
      InitAndRegisterMesh(malformed_gltf),
      ".*Failed parsing the on-disk glTF file.*property is missing[^]*");
}

// The error conditions for GltfMeshExtractor::FindTargetRootNodes().
TEST_F(RenderEngineGlTest, GltfFindTargetRootNodesErrors) {
  SetupGltfTest();

  // Out of range default scene index.
  const fs::path default_scene_oor = temp_dir_ / "default_scene_oor.gltf";
  TweakGltf(gltf_pyramid_path(), default_scene_oor,
            [](nlohmann::json* model_ptr) {
              nlohmann::json& model = *model_ptr;
              model["scene"] = 10;
            });
  DRAKE_EXPECT_THROWS_MESSAGE(InitAndRegisterMesh(default_scene_oor),
                              ".*has an invalid value for the .glTF.scene..*");

  // Scene references no nodes.
  const fs::path empty_scene = temp_dir_ / "empty_scene.gltf";
  TweakGltf(gltf_pyramid_path(), empty_scene, [](nlohmann::json* model_ptr) {
    nlohmann::json& model = *model_ptr;
    model["scenes"][0]["nodes"] = {};
  });
  DRAKE_EXPECT_THROWS_MESSAGE(InitAndRegisterMesh(empty_scene),
                              ".*scene 0 has no root nodes.*");

  // No nodes or scenes.
  const fs::path no_nodes_scenes = temp_dir_ / "no_nodes_scenes.gltf";
  TweakGltf(gltf_pyramid_path(), no_nodes_scenes,
            [](nlohmann::json* model_ptr) {
              nlohmann::json& model = *model_ptr;
              model.erase("nodes");
              model.erase("scenes");
              model.erase("scene");
            });
  DRAKE_EXPECT_THROWS_MESSAGE(InitAndRegisterMesh(no_nodes_scenes),
                              ".*no scenes and no nodes.*");

  // Node cycle (no scenes) --> there are no roots.
  const fs::path node_cycle = temp_dir_ / "node_cycle.gltf";
  TweakGltf(gltf_pyramid_path(), node_cycle, [](nlohmann::json* model_ptr) {
    nlohmann::json& model = *model_ptr;
    // Node is its own parent.
    model["nodes"][0]["children"].push_back(0);
    model.erase("scenes");
    model.erase("scene");
  });
  DRAKE_EXPECT_THROWS_MESSAGE(InitAndRegisterMesh(node_cycle),
                              ".*none of its 1 nodes are root nodes.*");
}

// The error conditions for GltfMeshExtractor::InstantiateMesh().
TEST_F(RenderEngineGlTest, GltfInstantiateMeshErrors) {
  SetupGltfTest();

  // All attributes reference the same buffer.
  const fs::path cross_buffer_attribs = temp_dir_ / "cross_buffer_attribs.gltf";
  TweakGltf(gltf_pyramid_path(), cross_buffer_attribs,
            [](nlohmann::json* model_ptr) {
              nlohmann::json& model = *model_ptr;
              DRAKE_DEMAND(model["bufferViews"][0]["buffer"].get<int>() == 0);
              model["bufferViews"][0]["buffer"] = 2;
            });
  DRAKE_EXPECT_THROWS_MESSAGE(InitAndRegisterMesh(cross_buffer_attribs),
                              ".*All attributes.*same buffer.*");
}

// The error conditions for GltfMeshExtractor::ConfigureIndexBuffer().
TEST_F(RenderEngineGlTest, GltfConfigureIndexBufferErrors) {
  SetupGltfTest();

  // Primitives must be indexed.
  const fs::path non_indexed_prim = temp_dir_ / "non_indexed_prim.gltf";
  TweakGltf(gltf_pyramid_path(), non_indexed_prim,
            [](nlohmann::json* model_ptr) {
              nlohmann::json& model = *model_ptr;
              model["meshes"][0]["primitives"][0].erase("indices");
            });
  DRAKE_EXPECT_THROWS_MESSAGE(InitAndRegisterMesh(non_indexed_prim),
                              ".*All meshes must be indexed.*");

  // Accessors for index arrays must have "SCALAR" type.
  const fs::path bad_index_type = temp_dir_ / "bad_index_type.gltf";
  TweakGltf(gltf_pyramid_path(), bad_index_type, [](nlohmann::json* model_ptr) {
    nlohmann::json& model = *model_ptr;
    const int a = model["meshes"][0]["primitives"][0]["indices"].get<int>();
    model["accessors"][a]["type"] = "VEC2";
  });
  DRAKE_EXPECT_THROWS_MESSAGE(
      InitAndRegisterMesh(bad_index_type),
      ".*type of an accessor for mesh indices to be 'SCALAR'.*");

  // Index buffer_view byteStride is 0.
  const fs::path bad_index_stride = temp_dir_ / "bad_index_stride.gltf";
  TweakGltf(gltf_pyramid_path(), bad_index_stride,
            [](nlohmann::json* model_ptr) {
              nlohmann::json& model = *model_ptr;
              const int accessor_index =
                  model["meshes"][0]["primitives"][0]["indices"].get<int>();
              const int bufferView_index =
                  model["accessors"][accessor_index]["bufferView"].get<int>();
              model["bufferViews"][bufferView_index]["byteStride"] = 4;
            });
  DRAKE_EXPECT_THROWS_MESSAGE(InitAndRegisterMesh(bad_index_stride),
                              ".*indices must be compactly stored.*");
}

// Tests RenderEngineGl's handling of embedded textures:
//
//  - An embedded texture for baseColorTexture gets used.
//  - Unused textures don't get handled at all.
//
// We'll replace the reference to an external base color texture with an
// embedded texture representation of the same image and then confirm a) that we
// get the same rendered image and b) the only image included in the engine's
// texture library is the embedded image.
//
// Note: external images are implicitly tested in every other glTF test as the
// renderings require the external image and its correct application.
TEST_F(RenderEngineGlTest, GltfEmbeddedTextures) {
  SetupGltfTest();

  const fs::path embedded_path = temp_dir_ / "embedded_texture.gltf";
  TweakGltf(gltf_pyramid_path(), embedded_path, [](nlohmann::json* model_ptr) {
    nlohmann::json& model = *model_ptr;

    // Reads the base64 encoding of the pyramid's color texture.
    std::ifstream data_file(
        FindResourceOrThrow("drake/geometry/render/test/meshes/"
                            "fully_textured_pyramid_base_color.base64"));
    DRAKE_DEMAND(data_file.good());
    std::stringstream data_contents;
    data_contents << data_file.rdbuf();
    // We know the decoded bytes from the .base64 file has this byte length.
    const int byte_length = 10501;

    // Adds a buffer with the embedded texture.
    const int buffer_index = ssize(model["buffers"]);
    model["buffers"].push_back(
        {{"byteLength", byte_length},
         {"uri", fmt::format("data:application/octet-stream;base64,{}",
                             std::move(data_contents).str())}});

    const int buffer_view_index = ssize(model["bufferViews"]);
    model["bufferViews"].push_back({{"buffer", buffer_index},
                                    {"byteLength", byte_length},
                                    {"byteOffset", 0}});

    // Creates an image based on the added buffer.
    const int image_index = ssize(model["images"]);
    model["images"].push_back({{"bufferView", buffer_view_index},
                               {"byteLength", byte_length},
                               {"mimeType", "image/png"},
                               {"name", "embedded_base_color.png"}});

    const int texture_index = ssize(model["textures"]);
    // Confirms that the glTF already had textures and the resulting glTF will
    // have multiple textures.
    DRAKE_DEMAND(texture_index > 0);
    model["textures"].push_back({{"sampler", 0}, {"source", image_index}});

    // Replaces all baseColorTextures with the new texture.
    bool replaced = false;
    for (auto& material : model["materials"]) {
      if (material.contains("pbrMetallicRoughness")) {
        auto& pbr = material["pbrMetallicRoughness"];
        if (pbr.contains("baseColorTexture")) {
          pbr["baseColorTexture"]["index"] = texture_index;
          replaced = true;
        }
      }
    }
    DRAKE_DEMAND(replaced);
  });

  DoCompareTest(embedded_path);

  RenderEngineGlTester tester(renderer_.get());
  const TextureLibrary& library = tester.texture_library();
  // We only have a single texture in the library. The embedded image we expect.
  // Also, we didn't copy any of the external textures to the temp directory;
  // if we'd tried using them, we would've had a file access violation.
  ASSERT_EQ(ssize(library.textures()), 1);
  ASSERT_TRUE(library.textures().begin()->first.starts_with(
      TextureLibrary::InMemoryPrefix()));
}

// Tests RenderEngineGl's node selection heuristics.
//
//  - If gltf.scene is not defined, use scene 0.
//  - If gltf.scenes is not defined, use all nodes.
TEST_F(RenderEngineGlTest, GltfIncludedNodes) {
  SetupGltfTest();

  // Remove the gltf.scene value.
  const fs::path no_default_scene = temp_dir_ / "no_default_scene.gltf";
  TweakGltf(gltf_pyramid_path(), no_default_scene,
            [](nlohmann::json* model_ptr) {
              nlohmann::json& model = *model_ptr;
              DRAKE_DEMAND(model.contains("scene"));
              model.erase("scene");
            });
  DoCompareTest(no_default_scene);

  // Remove the gltf.scene and gltf.scenes.
  const fs::path no_scenes = temp_dir_ / "no_scenes.gltf";
  TweakGltf(gltf_pyramid_path(), no_scenes, [](nlohmann::json* model_ptr) {
    nlohmann::json& model = *model_ptr;
    DRAKE_DEMAND(model.contains("scene"));
    model.erase("scene");
    DRAKE_DEMAND(model.contains("scenes"));
    model.erase("scenes");
  });
  DoCompareTest(no_scenes);
}

// Tests the composition of transforms in the node hierarchy. We'll take the
// pyramid model (which has a single node with no transforms). We'll inject a
// new node into the hierarchy above it and apply a novel transform and define
// the pyramid node's transform to be its inverse. The rendered result should
// be the same as the original.
//
// Specifically, we want to show:
//
//    1. When defined, translation, scale, and rotation all contribute to the
//       transform.
//    2. Scale can be anisotropic.
//    3. The transform can be given as a 4x4 homogeneous matrix.
//
// Note: by removing the gltf.scene and gltf.scenes, we are implicitly
// confirming that the correct root nodes are identified (in
// GltfMeshExtractor::FindAllRootNodes()).
TEST_F(RenderEngineGlTest, GltfNodeTransforms) {
  SetupGltfTest();

  // First show that translation, scale, and rotation all combine. Note:
  // we're using a *uniform* scale here because for a non-identity rotation and
  // non-uniform scale, there may be no transform between pyramid and injected
  // node that can undo the injected node's transformation.
  const fs::path from_xform_elements = temp_dir_ / "from_xform_elements.gltf";
  TweakGltf(
      gltf_pyramid_path(), from_xform_elements, [](nlohmann::json* model_ptr) {
        nlohmann::json& model = *model_ptr;
        DRAKE_DEMAND(model["nodes"].size() == 1);
        // Remove gltf.scene and gltf.scenes.
        model.erase("scene");
        model.erase("scenes");
        const double scale = 2;
        const double inv_scale = 1.0 / scale;
        const Eigen::Quaterniond q_F1{0.5, 0.5, 0.5, 0.5};
        const Eigen::Quaterniond q_10 = q_F1.inverse();
        const Vector3d p_F1{1, 2, 3};
        const Vector3d p_10 = inv_scale * (RotationMatrixd(q_10) * -p_F1);
        model["nodes"].push_back(
            {{"name", "injected"},
             {"children", {0}},
             {"translation", std::vector<double>(p_F1.data(), p_F1.data() + 3)},
             {"rotation", {q_F1.x(), q_F1.y(), q_F1.z(), q_F1.w()}},
             {"scale", {scale, scale, scale}}});
        auto& pyramid = model["nodes"][0];
        pyramid["translation"] =
            std::vector<double>(p_10.data(), p_10.data() + 3);
        pyramid["rotation"] = {q_10.x(), q_10.y(), q_10.z(), q_10.w()};
        pyramid["scale"] = {inv_scale, inv_scale, inv_scale};
        model["scenes"][0]["nodes"] = {1};
      });
  DoCompareTest(from_xform_elements);

  // Test against non-uniform scale.
  const fs::path non_uniform_scale = temp_dir_ / "non_uniform_scale.gltf";
  TweakGltf(
      gltf_pyramid_path(), non_uniform_scale, [](nlohmann::json* model_ptr) {
        nlohmann::json& model = *model_ptr;
        DRAKE_DEMAND(model["nodes"].size() == 1);
        model["nodes"].push_back(
            {{"name", "injected"}, {"children", {0}}, {"scale", {4, 2, 5}}});
        auto& pyramid = model["nodes"][0];
        pyramid["scale"] = {1.0 / 4.0, 1.0 / 2.0, 1.0 / 5.0};
        model["scenes"][0]["nodes"] = {1};
      });
  DoCompareTest(non_uniform_scale);

  // Test a general matrix. We can use non-uniform scale here because we don't
  // need to decompose T_10 into scale, rotation, and translation. A weird
  // skew matrix is fine, as long as it is equal to T_F1⁻¹.
  const fs::path transform_matrix = temp_dir_ / "transform_matrix.gltf";
  TweakGltf(
      gltf_pyramid_path(), transform_matrix, [](nlohmann::json* model_ptr) {
        nlohmann::json& model = *model_ptr;
        RigidTransformd X_F1(RotationMatrixd(), Vector3d(1, 2, 3));
        const Matrix4d T_F1 =
            DiagonalMatrix<double, 4>(4, 2, 5, 1) * X_F1.GetAsMatrix4();
        const Matrix4d T_10 = X_F1.inverse().GetAsMatrix4() *
                              DiagonalMatrix<double, 4>(0.25, 0.5, 0.2, 1);
        auto matrix_vector = [](const Matrix4d& T) {
          std::vector<double> result;
          for (int c = 0; c < 4; ++c) {
            for (int r = 0; r < 4; ++r) {
              // glTF uses column-major ordering.
              result.push_back(T(r, c));
            }
          }
          return result;
        };
        DRAKE_DEMAND(model["nodes"].size() == 1);
        model["nodes"].push_back({{"name", "injected"},
                                  {"children", {0}},
                                  {"matrix", matrix_vector(T_F1)}});
        auto& pyramid = model["nodes"][0];
        pyramid["matrix"] = matrix_vector(T_10);
        model["scenes"][0]["nodes"] = {1};
      });
  DoCompareTest(transform_matrix);
}

// Tests various requirements and behaviors relating to vertex attributes:
//
//   1. Missing normals throw.
//   2. Missing texture coordinates won't apply textures.
//   3. Textures that reference any uv set other than 0 are ignored.
TEST_F(RenderEngineGlTest, GltfVertexAttributes) {
  SetupGltfTest();

  // Missing normals are an error.
  const fs::path missing_normals = temp_dir_ / "missing_normals.gltf";
  TweakGltf(gltf_pyramid_path(), missing_normals,
            [](nlohmann::json* model_ptr) {
              nlohmann::json& model = *model_ptr;
              for (auto& mesh : model["meshes"]) {
                for (auto& primitive : mesh["primitives"]) {
                  primitive["attributes"].erase("NORMAL");
                }
              }
            });
  DRAKE_EXPECT_THROWS_MESSAGE(InitAndRegisterMesh(missing_normals),
                              ".* missing the attribute 'NORMAL'.*");

  // Missing uvs merely mean no texture gets applied.
  const fs::path missing_uvs = temp_dir_ / "missing_uvs.gltf";
  TweakGltf(gltf_pyramid_path(), missing_uvs, [](nlohmann::json* model_ptr) {
    nlohmann::json& model = *model_ptr;
    for (auto& mesh : model["meshes"]) {
      for (auto& primitive : mesh["primitives"]) {
        primitive["attributes"].erase("TEXCOORD_0");
      }
    }
  });
  // Registering it is not a problem. But no textures got registered.
  EXPECT_NO_THROW(InitAndRegisterMesh(missing_uvs));
  ASSERT_EQ(
      ssize(RenderEngineGlTester(renderer_.get()).texture_library().textures()),
      0);

  // A texture referencing the wrong uv set (non-zero) is ignored.
  const fs::path wrong_uv_set = temp_dir_ / "wrong_uv_set.gltf";
  TweakGltf(gltf_pyramid_path(), wrong_uv_set, [](nlohmann::json* model_ptr) {
    nlohmann::json& model = *model_ptr;
    for (auto& mat : model["materials"]) {
      auto& color_info = mat["pbrMetallicRoughness"]["baseColorTexture"];
      color_info["texCoord"] = 1;
    }
  });
  // Registering it is not a problem. But no textures got registered.
  EXPECT_NO_THROW(InitAndRegisterMesh(wrong_uv_set));
  ASSERT_EQ(
      ssize(RenderEngineGlTester(renderer_.get()).texture_library().textures()),
      0);
}

// Tests that the material heuristic is applied if the glTF mesh doesn't
// apply one explicitly.
//
// In this case, we'll simply set a diffuse texture in the perception properties
// and confirm its (sole) presence in the texture library as evidence that the
// fallback material logic has been engaged.
TEST_F(RenderEngineGlTest, GltfMaterialHeuristic) {
  SetupGltfTest();

  const fs::path no_material = temp_dir_ / "no_material.gltf";
  TweakGltf(gltf_pyramid_path(), no_material, [](nlohmann::json* model_ptr) {
    nlohmann::json& model = *model_ptr;
    for (auto& mesh : model["meshes"]) {
      for (auto& primitive : mesh["primitives"]) {
        primitive.erase("material");
      }
    }
  });

  // Register with diffuse map specified as perception properties.
  const std::string diffuse_map = FindResourceOrThrow(
      "drake/geometry/render/test/meshes/rainbow_stripes.png");
  Init(RigidTransformd::Identity());
  PerceptionProperties material;
  material.AddProperty("label", "id", RenderLabel(1));
  material.AddProperty("phong", "diffuse_map", diffuse_map);
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, Mesh(no_material.string()), material,
                            RigidTransformd::Identity(),
                            false /* needs update */);

  RenderEngineGlTester tester(renderer_.get());
  const TextureLibrary& library = tester.texture_library();
  // We only have a single texture in the library: the embedded image we expect.
  // Also, we didn't copy any of the external textures to the temp directory;
  // if we'd tried using them, we would've had a file access violation.
  ASSERT_EQ(ssize(library.textures()), 1);
  ASSERT_TRUE(
      library.textures().begin()->first.ends_with("rainbow_stripes.png"));
}

// The baseline regression test against a typical Drake-compatible glTF file.
// Also confirm that the functionality still works in the cloned engine.
TEST_F(RenderEngineGlTest, GltfBaseline) {
  InitAndRegisterMesh(gltf_pyramid_path());
  {
    SCOPED_TRACE("baseline");
    RenderAndCompareAgainstRef("gltf_baseline");
    // The only texture type currently supported is the base color.
    RenderEngineGlTester tester(renderer_.get());
    const TextureLibrary& library = tester.texture_library();
    ASSERT_EQ(ssize(library.textures()), 1);
    ASSERT_TRUE(library.textures().begin()->first.ends_with("base_color.png"));
  }

  {
    SCOPED_TRACE("from_clone");
    auto base_engine = renderer_->Clone();
    RenderAndCompareAgainstRef("from_clone", base_engine.get());
  }
}

// Confirm that a glTF buffer gets registered once and reused. We'll duplicate
// the primitive and confirm they both reference the same vertex buffer.
TEST_F(RenderEngineGlTest, GltfOpenGlBufferReuse) {
  SetupGltfTest();

  const fs::path multi_prim = temp_dir_ / "multi_prim.gltf";
  TweakGltf(gltf_pyramid_path(), multi_prim, [](nlohmann::json* model_ptr) {
    nlohmann::json& model = *model_ptr;
    auto& primitives = model["meshes"][0]["primitives"];
    primitives.push_back(primitives[0]);
  });
  InitAndRegisterMesh(multi_prim);

  RenderEngineGlTester tester(renderer_.get());
  const auto& geometries = tester.opengl_geometries();
  ASSERT_EQ(geometries.size(), 2);
  EXPECT_EQ(geometries[0].vertex_buffer, geometries[1].vertex_buffer);
}

// A variant of MeshTest. Confirms the support for mesh files which contain
// multiple materials/parts. Conceptually, the mesh file is a cube with
// different colors on each side. We'll render the cube six times with different
// orientations to confirm that each face renders as expected. The *structure*
// of the mesh file tests various aspects:
//
//  1. Multiple objects.
//  2. Multiple materials.
//  3. Some materials use diffuse color, some use map.
//  4. The texture is not vertically symmetric. So, if there's an inversion
//     problem, we'll get a bad face.
//
// If all of that is processed correctly, we should get a cube with a different
// color on each face. We'll test for those colors.
//
// This test renders against an original engine and its clone (to confirm that
// the render artifacts survive cloning). This has a secondary benefit of
// detecting if anything happens to corrupt the relationship between original
// and cloned context (see, e.g., #21326).
TEST_F(RenderEngineGlTest, MultiMaterialObj) {
  struct Face {
    // The expected *illuminated* material color. The simple illumination model
    // guarantees that the rendered color should be that of the material --
    // either the given Kd value *or* the map color at the test location.
    // TODO(20234): this will change to product of diffuse color and texture.
    Rgba rendered_color;
    RotationMatrixd rotation;
    std::string name;
  };
  Init(X_WR_, true);

  const std::string filename =
      FindResourceOrThrow("drake/geometry/render/test/meshes/rainbow_box.obj");

  Mesh mesh(filename);
  expected_label_ = RenderLabel(3);
  // Note: Passing diffuse color or texture to mesh with materials spawns a
  // warning.
  PerceptionProperties material;
  material.AddProperty("label", "id", expected_label_);
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, mesh, material, RigidTransformd::Identity(),
                            true /* needs update */);

  const std::vector<Face> faces{
      {.rendered_color = Rgba(0.016, 0.945, 0.129),
       .rotation = RotationMatrixd(),
       .name = "green"},
      {.rendered_color = Rgba(0.8, 0.359, 0.023),
       .rotation = RotationMatrixd::MakeXRotation(M_PI / 2),
       .name = "orange"},
      {.rendered_color = Rgba(0.945, 0.016, 0.016),
       .rotation = RotationMatrixd::MakeXRotation(M_PI),
       .name = "red"},
      {.rendered_color = Rgba(0.098, 0.016, 0.945),
       .rotation = RotationMatrixd::MakeXRotation(-M_PI / 2),
       .name = "blue"},
      {.rendered_color = Rgba(0.799, 0.8, 0),
       .rotation = RotationMatrixd::MakeYRotation(-M_PI / 2),
       .name = "yellow"},
      {.rendered_color = Rgba(0.436, 0, 0.8),
       .rotation = RotationMatrixd::MakeYRotation(M_PI / 2),
       .name = "purple"},
  };

  // Render from the original to make sure it's complete and correct.
  for (const auto& face : faces) {
    SCOPED_TRACE(
        fmt::format("multi-material test on {} face - original", face.name));
    expected_color_ = face.rendered_color;

    renderer_->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
        {id, RigidTransformd(face.rotation)}});
    PerformCenterShapeTest(renderer_.get());
  }

  // Repeat that from a clone to confirm that the artifacts survived cloning.
  std::unique_ptr<RenderEngine> clone = renderer_->Clone();
  auto* gl_clone = dynamic_cast<RenderEngineGl*>(clone.get());
  for (const auto& face : faces) {
    SCOPED_TRACE(
        fmt::format("multi-material test on {} face - clone", face.name));
    expected_color_ = face.rendered_color;

    gl_clone->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
        {id, RigidTransformd(face.rotation)}});
    PerformCenterShapeTest(gl_clone);
  }

  // Confirm all parts get removed when removing the geometry.
  gl_clone->RemoveGeometry(id);
  expected_color_ = kTerrainColor;
  expected_object_depth_ = expected_outlier_depth_;
  expected_label_ = expected_outlier_label_;
  for (const auto& face : faces) {
    SCOPED_TRACE(fmt::format("multi-material test on {} face - after removal",
                             face.name));

    gl_clone->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
        {id, RigidTransformd(face.rotation)}});
    PerformCenterShapeTest(gl_clone);
  }
}

// For the Convex shape, we're confirming that the convex hull gets rendered
// and not the shape with a hole.
TEST_F(RenderEngineGlTest, ConvexTest) {
  Init(X_WR_, true);

  // Note: it is expected that in addition to the hole, this file does not have
  // normals. Therefore, if it were processed with the Mesh logic, this test
  // would throw complaining about missing normals.
  auto filename = FindResourceOrThrow(
      "drake/examples/scene_graph/cuboctahedron_with_hole.obj");

  // It's important to instantiate a *scaled* Convex because of how
  // RenderEngineGl caches convex hulls. We'll detect that it got cached and
  // instantiated as expected by examining the depth return.
  //
  // The cuboctahedron is bound by an aligned bounding box that extends to
  // +/-1 along each axis. Scaling it by 0.5 means the box will only extend to
  // 0.5 in each direction. The top has moved down 0.5 m which increases the
  // depth value by 0.5.
  Convex convex(filename, 0.5);
  expected_object_depth_ += 0.5;
  expected_label_ = RenderLabel(4);

  PerceptionProperties material = simple_material();
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, convex, material, RigidTransformd::Identity(),
                            true /* needs update */);
  renderer_->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
      {id, RigidTransformd::Identity()}});

  SCOPED_TRACE("Convex test");
  PerformCenterShapeTest(renderer_.get());
}

// Confirms that Meshes referencing a file with an unsupported extension are
// ignored. (There's also an untested one-time warning.)
// This doesn't include Convex, because Convex support is predicated on whether
// we can compute a convex hull for the named file -- that is tested elsewhere.
TEST_F(RenderEngineGlTest, UnsupportedMeshFileType) {
  Init(X_WR_, false);
  const PerceptionProperties material = simple_material();
  const GeometryId id = GeometryId::get_new_id();

  const Mesh mesh("invalid.fbx");
  EXPECT_FALSE(renderer_->RegisterVisual(id, mesh, material,
                                         RigidTransformd::Identity(),
                                         false /* needs update */));
}

// Performs the test to cast textures to uchar channels. It depends on the image
// with non-uchar channels being converted to uchar channels losslessly. An
// uint16 image is loaded to prove the existence of the conversion, but this
// test doesn't guarantee universal conversion success.
TEST_F(RenderEngineGlTest, NonUcharChannelTextures) {
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
    RenderEngineGl renderer;
    InitializeRenderer(X_WR_, false /* add terrain */, &renderer);

    const GeometryId id = GeometryId::get_new_id();
    PerceptionProperties props = simple_material(true);
    renderer.RegisterVisual(id, box, props, RigidTransformd::Identity(), true);
    renderer.RenderColorImage(camera, &color_uchar_texture);
  }

  // Render a box with an uint16-channel PNG texture.
  ImageRgba8U color_uint16_texture(intrinsics.width(), intrinsics.height());
  {
    RenderEngineGl renderer;
    InitializeRenderer(X_WR_, false /* add terrain */, &renderer);

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
      unordered_map<GeometryId, RigidTransformd>{{sphere_id_, X_WT_new}});
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
  const render::ClippingRange& ref_clipping = ref_core.clipping();
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
  PerceptionProperties material;
  material.AddProperty("label", "id", RenderLabel::kDontCare);
  renderer_->RegisterVisual(id, box, material, RigidTransformd::Identity(),
                            true);
  RigidTransformd X_WV{Translation3d(0, 0, 0.5)};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});

  EXPECT_NO_THROW(Render());
}

// Tests that RenderEngineGl's default render label is kDontCare.
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

  // The engine's default is "don't care".
  RenderEngineGl renderer;
  InitializeRenderer(X_WR_, true /* add terrain */, &renderer);

  DRAKE_EXPECT_NO_THROW(populate_default_sphere(&renderer));
  expected_label_ = RenderLabel::kDontCare;
  expected_color_ = RgbaColor(renderer.parameters().default_diffuse);

  PerformCenterShapeTest(&renderer);
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

// We need to confirm that two Convex shapes, referring to the same file name,
// share the same underlying cached geometry.
TEST_F(RenderEngineGlTest, ConvexGeometryReuse) {
  RenderEngineGl engine;
  RenderEngineGlTester tester(&engine);

  auto filename = FindResourceOrThrow(
      "drake/examples/scene_graph/cuboctahedron_with_hole.obj");

  auto add_convex = [&filename, &engine](double scale) {
    const GeometryId id = GeometryId::get_new_id();
    PerceptionProperties material;
    material.AddProperty("label", "id", RenderLabel(17));
    const bool accepted = engine.RegisterVisual(id, Convex(filename, scale),
                                                material, RigidTransformd());
    DRAKE_DEMAND(accepted);
    return id;
  };

  const GeometryId id1 = add_convex(0.5);
  const GeometryId id2 = add_convex(1.5);
  const RenderEngineGlTester::Prop& prop1 = tester.GetVisual(id1);
  const RenderEngineGlTester::Prop& prop2 = tester.GetVisual(id2);

  EXPECT_EQ(prop1.instances.size(), 1);
  EXPECT_EQ(prop1.instances.size(), prop2.instances.size());
  // Different instances nevertheless share the same geometry.
  EXPECT_NE(&prop1.instances[0], &prop2.instances[0]);
  EXPECT_EQ(prop1.instances[0].geometry, prop2.instances[0].geometry);
}

// Confirms that when requesting the same mesh multiple times, only a single
// OpenGlGeometry is produced.
TEST_F(RenderEngineGlTest, MeshGeometryReuse) {
  RenderEngineGl engine;
  RenderEngineGlTester tester(&engine);

  auto filename =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj");

  auto add_mesh = [&filename, &engine]() {
    const GeometryId id = GeometryId::get_new_id();
    PerceptionProperties material;
    material.AddProperty("label", "id", RenderLabel(17));
    const bool accepted =
        engine.RegisterVisual(id, Mesh(filename), material, RigidTransformd());
    DRAKE_DEMAND(accepted);
    return id;
  };

  const GeometryId id1 = add_mesh();
  const GeometryId id2 = add_mesh();
  const RenderEngineGlTester::Prop& prop1 = tester.GetVisual(id1);
  const RenderEngineGlTester::Prop& prop2 = tester.GetVisual(id2);

  EXPECT_EQ(prop1.instances.size(), 1);
  EXPECT_EQ(prop1.instances.size(), prop2.instances.size());
  // Different instances nevertheless share the same geometry.
  EXPECT_NE(&prop1.instances[0], &prop2.instances[0]);
  EXPECT_EQ(prop1.instances[0].geometry, prop2.instances[0].geometry);
}

// Confirm the properties of the fallback camera using the following
// methodology:
//
// Create a scene with a box above a ground plane. The box is parallel with the
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
TEST_F(RenderEngineGlTest, FallbackLight) {
  const RenderEngineGlParams params{.default_clear_color = kBgColor};
  RenderEngineGl renderer(params);

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
  for (const auto& config : configs) {
    SCOPED_TRACE(config.description);
    renderer.UpdateViewpoint(config.X_WR);

    EXPECT_NO_THROW(renderer.RenderColorImage(camera, &image));

    // We test the images by looking at the colors along a row on the bottom of
    // the image and near the middle of the image. We won't do the top because
    // in view B, the clipped plane reveals the background color.
    //
    // Typically, if one pixel is wrong, many pixels are wrong. So, we use this
    // atypical test spelling to prevent pixel spam for failure. One bad pixel
    // is enough.
    const int mid_height = image.height() / 2;
    for (int c = 0; c < image.width(); ++c) {
      for (int r : {0, mid_height}) {
        RgbaColor dut(image.at(c, r));
        if (!IsColorNear(dut, config.expected_color)) {
          EXPECT_EQ(dut, config.expected_color)
              << "image color: " << dut << "\n"
              << "expected color: " << config.expected_color << "\n"
              << "at pixel (" << c << ", " << r << ")";
          break;
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
// This test does *not* test the subtle distinctions between the lights such as
// point light and spotlight have intensity fall off as the normal no longer
// points toward the light. These gross lighting properties should be
// immediately apparent in any rendering.
TEST_F(RenderEngineGlTest, SingleLight) {
  struct Config {
    LightParameter light;
    RgbaColor expected_color;
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
  const Rgba modulated_color = kTerrainColor * light_color;
  const Rgba low_intensity_color(kTerrainColor.r() * 0.1,
                                 kTerrainColor.g() * 0.1,
                                 kTerrainColor.b() * 0.1);  // Alpha = 1.0.

  // We'll omit the light type to save space, setting it once in the test loop.

  // The baseline configuration implicitly tests white light, intensity = 1,
  // no attenuation (1, 0, 0), and transformation from camera to world frame of
  // both position and direction of the light. Note, light direction is
  // *intentionally* specified with non-unit vectors to confirm that the
  // render engine takes responsibility for normalizing.
  const std::vector<Config> configs{
      {.light = {.color = Rgba(1, 1, 1),
                 .attenuation_values = {1, 0, 0},
                 .position = {0, 0, 0},
                 .frame = "camera",
                 .intensity = 1.0,
                 .direction = {0, 0, 0.5},
                 // If you show the window for spotlight images, the spotlight
                 // circle will exactly fit from image top to bottom.
                 .cone_angle = 22.5},
       .expected_color = kTerrainColor,
       .description = "Baseline posed in camera"},
      {.light = {.color = Rgba(1, 1, 1),
                 .attenuation_values = {1, 0, 0},
                 .position = {0, 0, dist},
                 .frame = "world",
                 .intensity = 1.0,
                 .direction = {0, 0, -2},
                 .cone_angle = 22.5},
       .expected_color = kTerrainColor,
       // Should be identical to the baseline image.
       .description = "Baseline posed in camera"},
      {.light = {.color = Rgba(1, 1, 1),
                 .attenuation_values = {1, 0, 0},
                 .position = {0, 0, dist},
                 .frame = "camera",
                 .intensity = 1.0,
                 .direction = {0, 0, -0.01},
                 .cone_angle = 22.5},
       .expected_color = Rgba(0, 0, 0),
       // The lights are positioned badly to illuminate anything.
       .description = "Camera coordinates in the world frame - nothing lit!"},
      {.light = {.color = light_color, .cone_angle = 22.5},
       .expected_color = modulated_color,
       .description = "Non-white light color"},
      {.light = {.intensity = 0.1, .cone_angle = 22.5},
       .expected_color = kTerrainColor.scale_rgb(0.1),
       .description = "Low intensity"},
      {.light = {.intensity = 3.0, .cone_angle = 22.5},
       .expected_color = kTerrainColor.scale_rgb(3),
       .description = "High intensity"},
      {.light = {.attenuation_values = {2, 0, 0}, .cone_angle = 22.5},
       .expected_color = kTerrainColor.scale_rgb(0.5),
       .description = "Non-unit constant attenuation"},
      {.light = {.attenuation_values = {0, 1, 0}, .cone_angle = 22.5},
       .expected_color = kTerrainColor.scale_rgb(1 / dist),
       .description = "Linear attenuation"},
      {.light = {.attenuation_values = {0, 0, 1}, .cone_angle = 22.5},
       .expected_color = kTerrainColor.scale_rgb(1 / (dist * dist)),
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
      SCOPED_TRACE(
          fmt::format("{} - {}", fmt_streamed(l_type), config.description));
      LightParameter test_light = config.light;
      test_light.type = l_type;
      const RenderEngineGlParams params{.lights = {test_light}};
      RenderEngineGl renderer(params);

      InitializeRenderer(X_WR, true /* add terrain */, &renderer);

      EXPECT_NO_THROW(renderer.RenderColorImage(camera, &image));

      const RgbaColor test_color(image.at(cx, cy));
      EXPECT_TRUE(IsColorNear(test_color, config.expected_color))
          << "  test color: " << test_color << "\n"
          << "  expected color: " << config.expected_color;
    }
  }
}

// Quick test to make sure that lights combine. Also, confirm that too many
// lights throw.
TEST_F(RenderEngineGlTest, MultiLights) {
  // Too many lights throw.
  {
    const RenderEngineGlParams params{.lights = {
                                          {.position = {0, 0, 0}},
                                          {.position = {1, 0, 0}},
                                          {.position = {2, 0, 0}},
                                          {.position = {3, 0, 0}},
                                          {.position = {4, 0, 0}},
                                          {.position = {5, 0, 0}},
                                      }};
    auto make_renderer = [](const RenderEngineGlParams& p) {
      return RenderEngineGl(p);
    };
    EXPECT_THROW(make_renderer(params), std::exception);
  }

  // Lights combine.
  {
    const ColorRenderCamera camera(depth_camera_.core(), FLAGS_show_window);
    const RigidTransformd X_WR(RotationMatrixd::MakeXRotation(M_PI),
                               Vector3d(0, 0, 3));
    ImageRgba8U image(camera.core().intrinsics().width(),
                      camera.core().intrinsics().height());
    const int cx = image.width() / 2;
    const int cy = image.height() / 2;

    // The lights are pointing directly at the image center, but their total
    // intensity is 0.75. So, we should get 75% of the diffuse color.
    const RenderEngineGlParams params{
        .lights = {{.type = "point", .intensity = 0.25},
                   {.type = "spot", .intensity = 0.25, .cone_angle = 45},
                   {.type = "directional", .intensity = 0.25}}};
    RenderEngineGl renderer(params);

    InitializeRenderer(X_WR, true /* add terrain */, &renderer);

    EXPECT_NO_THROW(renderer.RenderColorImage(camera, &image));

    const RgbaColor test_color(image.at(cx, cy));
    const RgbaColor expected_color = kTerrainColor.scale_rgb(0.75);
    EXPECT_TRUE(IsColorNear(test_color, expected_color))
        << "  test color: " << test_color << "\n"
        << "  expected color: " << expected_color;
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
  const RgbaColor ground(kTerrainColor);
  const RgbaColor curr(curr_pixel);
  const RgbaColor next(next_pixel);

  const bool curr_is_ground = IsColorNear(curr, ground);
  const bool next_is_ground = IsColorNear(next, ground);
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

}  // namespace
}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
