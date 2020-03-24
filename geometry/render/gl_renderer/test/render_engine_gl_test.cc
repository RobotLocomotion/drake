#include "drake/geometry/render/gl_renderer/render_engine_gl.h"

#include <array>
#include <unordered_map>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/gl_renderer/render_engine_gl_factory.h"
#include "drake/geometry/render/render_label.h"
#include "drake/systems/sensors/color_palette.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render {
namespace gl {
namespace {

using Eigen::Translation3d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransformd;
using std::make_unique;
using std::unique_ptr;
using std::unordered_map;
using systems::sensors::Color;
using systems::sensors::ColorI;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::InvalidDepth;

// Default camera properties.
const int kWidth = 640;
const int kHeight = 480;
const double kZNear = 0.5;
const double kZFar = 5.;
const double kFovY = M_PI_4;
const bool kShowWindow = false;

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
const double kDepthTolerance = 2.5e-4;  // meters.

// Provide a default visual color for this tests -- it is intended to be
// different from the default color of the OpenGL render engine.
const ColorI kDefaultVisualColor = {229u, 229u, 229u};
const float kDefaultDistance{3.f};

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

class RenderEngineGlTest : public ::testing::Test {
 public:
  RenderEngineGlTest()
      : depth_(kWidth, kHeight),
        label_(kWidth, kHeight),
        // Looking straight down from 3m above the ground.
        X_WR_(Translation3d(0, 0, kDefaultDistance) *
              Eigen::AngleAxisd(M_PI, Vector3d::UnitY()) *
              Eigen::AngleAxisd(-M_PI_2, Vector3d::UnitZ())),
        geometry_id_(GeometryId::get_new_id()) {}

 protected:
  // Method to allow the normal case (render with the built-in renderer against
  // the default camera) to the member images with default window visibility.
  // This interface allows that to be completely reconfigured by the calling
  // test.
  void Render(RenderEngineGl* renderer = nullptr,
              const DepthCameraProperties* camera_in = nullptr,
              ImageDepth32F* depth_in = nullptr,
              ImageLabel16I* label_in = nullptr) {
    if (!renderer) renderer = renderer_.get();
    const DepthCameraProperties& camera = camera_in ? *camera_in : camera_;
    ImageLabel16I* label = label_in ? label_in : &label_;
    ImageDepth32F* depth = depth_in ? depth_in : &depth_;
    EXPECT_NO_THROW(renderer->RenderDepthImage(camera, depth));
    EXPECT_NO_THROW(renderer->RenderLabelImage(camera, kShowWindow, label));
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

  // Report the coordinates of the set of "outlier" pixels for a given set of
  // camera properties. These are the pixels that are drawn on by the
  // background (possibly flat plane, possibly sky) and not the primary
  // geometry.
  static std::vector<ScreenCoord> GetOutliers(const CameraProperties& camera) {
    return std::vector<ScreenCoord>{
        {kInset, kInset},
        {kInset, camera.height - kInset - 1},
        {camera.width - kInset - 1, camera.height - kInset - 1},
        {camera.width - kInset - 1, kInset}};
  }

  // Compute the coordinate of the pixel that is drawn on by the primary
  // geometry given a set of camera properties.
  static ScreenCoord GetInlier(const CameraProperties& camera) {
    return ScreenCoord(camera.width / 2, camera.height / 2);
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
      float delta = std::abs(expected_depth - actual_depth);
      if (delta <= tolerance) {
        return ::testing::AssertionSuccess();
      } else {
        return ::testing::AssertionFailure()
               << "Expected depth at " << coord << " to be " << expected_depth
               << ". Found " << actual_depth << ". Difference " << delta
               << "is greater than tolerance " << tolerance;
      }
    }
  }

  // Verifies the "outlier" pixels for the given camera belong to the terrain.
  // If images are provided, the given images will be tested, otherwise the
  // member images will be tested.
  void VerifyOutliers(const RenderEngineGl& renderer,
                      const DepthCameraProperties& camera, const char* name,
                      ImageDepth32F* depth_in = nullptr,
                      ImageLabel16I* label_in = nullptr) {
    ImageDepth32F& depth = depth_in ? *depth_in : depth_;
    ImageLabel16I& label = label_in ? *label_in : label_;

    for (const auto& screen_coord : GetOutliers(camera)) {
      EXPECT_TRUE(IsExpectedDepth(depth, screen_coord, expected_outlier_depth_,
                                  kDepthTolerance))
          << "Depth at: " << screen_coord << " for test: " << name;
      EXPECT_EQ(label.at(screen_coord.x, screen_coord.y)[0],
                expected_outlier_label_)
          << "Label at: " << screen_coord << " for test: " << name;
    }
  }

  void SetUp() override {}

  // All tests on this class must invoke this first.
  void SetUp(const RigidTransformd& X_WR, bool add_terrain = false) {
    renderer_ = make_unique<RenderEngineGl>();
    renderer_->UpdateViewpoint(X_WR);

    if (add_terrain) {
      const GeometryId ground_id = GeometryId::get_new_id();
      PerceptionProperties material;
      material.AddProperty("label", "id", RenderLabel::kDontCare);
      renderer_->RegisterVisual(ground_id, HalfSpace(), material,
                                RigidTransformd::Identity(),
                                false /** needs update */);
    }
  }

  // Creates a simple perception properties set for fixed, known results.
  PerceptionProperties simple_material() const {
    PerceptionProperties material;
    Vector4d color(kDefaultVisualColor.r / 255., kDefaultVisualColor.g / 255.,
                   kDefaultVisualColor.b / 255., 1.);
    material.AddProperty("phong", "diffuse", color);
    material.AddProperty("label", "id", expected_label_);
    return material;
  }

  // Populates the given renderer with the sphere required for
  // PerformCenterShapeTest().
  void PopulateSphereTest(RenderEngineGl* renderer) {
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

  // Performs the work to test the rendering with a sphere centered in the
  // image. To pass, the renderer will have to have been populated with a
  // compliant sphere and camera configuration (e.g., PopulateSphereTest()).
  // If force_hidden is true, then the render windows will be suppressed
  // regardless of any other settings.
  void PerformCenterShapeTest(RenderEngineGl* renderer, const char* name,
                              const DepthCameraProperties* camera = nullptr) {
    const DepthCameraProperties& cam = camera ? *camera : camera_;
    // Can't use the member images in case the camera has been configured to a
    // different size than the default camera_ configuration.
    ImageDepth32F depth(cam.width, cam.height);
    ImageLabel16I label(cam.width, cam.height);
    Render(renderer, &cam, &depth, &label);

    VerifyOutliers(*renderer, cam, name, &depth, &label);

    // Verifies inside the sphere.
    const ScreenCoord inlier = GetInlier(cam);

    EXPECT_TRUE(
        IsExpectedDepth(depth, inlier, expected_object_depth_, kDepthTolerance))
        << "Depth at: " << inlier << " for test: " << name;
    EXPECT_EQ(label.at(inlier.x, inlier.y)[0],
              static_cast<int>(expected_label_))
        << "Label at: " << inlier << " for test: " << name;
  }

  float expected_outlier_depth_{3.f};
  float expected_object_depth_{2.f};
  RenderLabel expected_label_;
  RenderLabel expected_outlier_label_{RenderLabel::kDontCare};
  RgbaColor default_color_{kDefaultVisualColor, 255};

  const DepthCameraProperties camera_ = {kWidth, kHeight, kFovY,
                                         "n/a",  kZNear,  kZFar};

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
  SetUp(RigidTransformd::Identity());
  Render();

  VerifyUniformLabel(RenderLabel::kEmpty);
  VerifyUniformDepth(std::numeric_limits<float>::infinity());
}

// Tests an image with *only* terrain (perpendicular to the camera's forward
// direction) -- no use of "inlier" or "outlier" pixel locations.
TEST_F(RenderEngineGlTest, TerrainTest) {
  SetUp(X_WR_, true);

  // At two different distances.
  Vector3d p_WR = X_WR_.translation();
  for (auto depth : std::array<float, 2>({{2.f, 4.9999f}})) {
    p_WR.z() = depth;
    X_WR_.set_translation(p_WR);
    renderer_->UpdateViewpoint(X_WR_);
    Render();
    VerifyUniformLabel(RenderLabel::kDontCare);
    VerifyUniformDepth(depth);
  }

  // Closer than kZNear.
  p_WR.z() = kZNear - 1e-5;
  X_WR_.set_translation(p_WR);
  renderer_->UpdateViewpoint(X_WR_);
  Render();
  VerifyUniformLabel(RenderLabel::kDontCare);
  VerifyUniformDepth(InvalidDepth::kTooClose);

  // Farther than kZFar.
  p_WR.z() = kZFar + 1e-3;
  X_WR_.set_translation(p_WR);
  renderer_->UpdateViewpoint(X_WR_);
  Render();
  VerifyUniformLabel(RenderLabel::kDontCare);
  // Verifies depth.
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      ASSERT_EQ(InvalidDepth::kTooFar, depth_.at(x, y)[0]);
    }
  }
}

// Performs the shape centered in the image with a box.
TEST_F(RenderEngineGlTest, BoxTest) {
  SetUp(X_WR_, true);

  // Sets up a box.
  Box box(1, 1, 1);
  expected_label_ = RenderLabel(1);
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, box, simple_material(),
                            RigidTransformd::Identity(),
                            true /* needs update */);
  RigidTransformd X_WV{Translation3d(0, 0, 0.5)};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});

  PerformCenterShapeTest(renderer_.get(), "Box test");
}

// Performs the shape centered in the image with a sphere.
TEST_F(RenderEngineGlTest, SphereTest) {
  SetUp(X_WR_, true);

  PopulateSphereTest(renderer_.get());

  PerformCenterShapeTest(renderer_.get(), "Sphere test");
}

// Performs the shape centered in the image with a cylinder.
TEST_F(RenderEngineGlTest, CylinderTest) {
  SetUp(X_WR_, true);

  // Sets up a cylinder.
  Cylinder cylinder(0.2, 1.2);
  expected_label_ = RenderLabel(2);
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, cylinder, simple_material(),
                            RigidTransformd::Identity(),
                            true /* needs update */);
  // Position the top of the cylinder to be 1 m above the terrain.
  RigidTransformd X_WV{Translation3d(0, 0, 0.4)};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});

  PerformCenterShapeTest(renderer_.get(), "Cylinder test");
}

// Performs the shape centered in the image with a mesh (which happens to be a
// box). This simultaneously confirms that if a diffuse_map is specified but it
// doesn't refer to a file that can be read, that the appearance defaults to
// the diffuse rgba value.
TEST_F(RenderEngineGlTest, MeshTest) {
  SetUp(X_WR_, true);

  auto filename = FindResourceOrThrow(
      "drake/systems/sensors/test/models/meshes/box.obj");
  Mesh mesh(filename);
  expected_label_ = RenderLabel(3);
  PerceptionProperties material = simple_material();
  // NOTE: Specifying a diffuse map with a known bad path, will force the box
  // to get the diffuse RGBA value (otherwise it would pick up the `box.png`
  // texture.
  material.AddProperty("phong", "diffuse_map", "bad_path");
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, mesh, material, RigidTransformd::Identity(),
                            true /* needs update */);
  renderer_->UpdatePoses(unordered_map<GeometryId, RigidTransformd>{
      {id, RigidTransformd::Identity()}});

  PerformCenterShapeTest(renderer_.get(), "Mesh test");
}

// Performs the shape centered in the image with a convex mesh (which happens to
// be a  box). This simultaneously confirms that if a diffuse_map is specified
// but it doesn't refer to a file that can be read, that the appearance defaults
// to the diffuse rgba value.
TEST_F(RenderEngineGlTest, ConvexTest) {
  SetUp(X_WR_, true);

  auto filename = FindResourceOrThrow(
      "drake/systems/sensors/test/models/meshes/box.obj");
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
TEST_F(RenderEngineGlTest, RemoveVisual) {
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());
  RenderLabel default_label = expected_label_;
  const float default_depth = expected_object_depth_;

  // Positions a sphere centered at <0, 0, z> with the given color.
  auto add_sphere = [this](const double z, const GeometryId geometry_id) {
    const double kRadius = 0.5;
    Sphere sphere{kRadius};
    const float depth = kDefaultDistance - kRadius - z;
    RenderLabel label = RenderLabel(5);
    PerceptionProperties material;
    material.AddProperty("label", "id", label);
    // This will accept all registered geometries and therefore, (bool)index
    // should always be true.
    renderer_->RegisterVisual(geometry_id, sphere, material,
                              RigidTransformd::Identity());
    RigidTransformd X_WV{Translation3d(0, 0, z)};
    X_WV_.insert({geometry_id, X_WV});
    renderer_->UpdatePoses(X_WV_);
    return std::make_tuple(label, depth);
  };

  // Sets the expected values prior to calling PerformCenterShapeTest().
  auto set_expectations = [this](const float depth, const RenderLabel& label) {
    expected_label_ = label;
    expected_object_depth_ = depth;
  };

  // Add another sphere of a different color in front of the default sphere.
  const GeometryId id1 = GeometryId::get_new_id();
  float depth1{};
  RenderLabel label1{};
  std::tie(label1, depth1) = add_sphere(0.75, id1);
  set_expectations(depth1, label1);
  PerformCenterShapeTest(renderer_.get(), "Second sphere added in remove test");

  // Add a _third_ sphere in front of the second.
  const GeometryId id2 = GeometryId::get_new_id();
  float depth2{};
  RenderLabel label2{};
  std::tie(label2, depth2) = add_sphere(1.0, id2);
  set_expectations(depth2, label2);
  PerformCenterShapeTest(renderer_.get(), "Third sphere added in remove test");

  // Remove the first sphere added; should report "true" and the render test
  // should pass without changing expectations.
  bool removed = renderer_->RemoveGeometry(id1);
  EXPECT_TRUE(removed);
  PerformCenterShapeTest(renderer_.get(), "First added sphere removed");

  // Remove the second added sphere; should report true and rendering should
  // return to its default configuration.
  removed = renderer_->RemoveGeometry(id2);
  EXPECT_TRUE(removed);
  set_expectations(default_depth, default_label);
  PerformCenterShapeTest(renderer_.get(),
                         "Default image restored by removing extra geometries");
}

// All of the clone tests use the PerformCenterShapeTest() with the sphere setup
// to confirm that the clone is behaving as anticipated.

// Tests that the cloned renderer produces the same images (i.e., passes the
// same test).
TEST_F(RenderEngineGlTest, SimpleClone) {
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  unique_ptr<RenderEngine> clone = renderer_->Clone();
  EXPECT_NE(dynamic_cast<RenderEngineGl*>(clone.get()), nullptr);
  PerformCenterShapeTest(dynamic_cast<RenderEngineGl*>(clone.get()),
                         "Simple clone");
}

// Tests that the cloned renderer still works, even when the original is
// deleted.
TEST_F(RenderEngineGlTest, ClonePersistence) {
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  unique_ptr<RenderEngine> clone = renderer_->Clone();
  // This causes the original renderer copied from to be destroyed.
  renderer_.reset();
  ASSERT_EQ(nullptr, renderer_);
  PerformCenterShapeTest(static_cast<RenderEngineGl*>(clone.get()),
                         "Clone persistence");
}

// Tests that the cloned renderer still works, even when the original has values
// RenderEngineGlTest
TEST_F(RenderEngineGlTest, CloneIndependence) {
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  unique_ptr<RenderEngine> clone = renderer_->Clone();
  // Move the terrain *up* 10 units in the z.
  RigidTransformd X_WT_new{Translation3d{0, 0, 10}};
  // This assumes that the terrain is zero-indexed.
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{geometry_id_, X_WT_new}});
  PerformCenterShapeTest(dynamic_cast<RenderEngineGl*>(clone.get()),
                         "Clone independence");
}

// Confirm that the renderer can be used for cameras with different properties.
// I.e., the camera intrinsics are defined *outside* the renderer.
TEST_F(RenderEngineGlTest, DifferentCameras) {
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

    // Test rendering with a previous camera once again to make sure the
    // existing target is reused correctly despite being of different sizes.
    PerformCenterShapeTest(renderer_.get(), "Camera change - small camera",
                           &small_camera);
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
    PerformCenterShapeTest(renderer_.get(), "Camera change - z near clips mesh",
                           &clipping_near_plane);
  }
}

// Tests that registered geometry without specific values renders without error
// except for default render labels.
// TODO(SeanCurtis-TRI): When the ability to set defaults is exposed through a
// public API, actually test for the *default values*. Until then, error-free
// rendering is sufficient.
TEST_F(RenderEngineGlTest, DefaultProperties) {
  SetUp(X_WR_, false /* no terrain */);

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

// This is a test to make sure that independent renders are truly independent.
// So, we're going to do a bunch of interleaved operations on the two renderers
// to make sure that each is still correct.
TEST_F(RenderEngineGlTest, MultipleRenderers) {
  RenderEngineGl engine1;
  RenderEngineGl engine2;
  const GeometryId ground = GeometryId::get_new_id();

  auto add_terrain = [ground](auto* renderer) {
    PerceptionProperties material;
    material.AddProperty("label", "id", RenderLabel::kDontCare);
    renderer->RegisterVisual(ground, HalfSpace(), material,
                             RigidTransformd::Identity(),
                             false /** needs update */);
  };

  engine1.UpdateViewpoint(X_WR_);
  add_terrain(&engine1);
  PopulateSphereTest(&engine1);
  PerformCenterShapeTest(&engine1, "engine1 - initial render");

  Render(&engine2);
  VerifyUniformDepth(std::numeric_limits<float>::infinity());

  engine2.UpdateViewpoint(X_WR_);
  add_terrain(&engine2);
  Box box(1, 1, 1);
  const GeometryId box_id = GeometryId::get_new_id();
  engine2.RegisterVisual(box_id, box, simple_material(),
                         RigidTransformd::Identity(), true);
  RigidTransformd X_WV{Translation3d(0.2, 0.2, 0.5)};
  engine2.UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{box_id, X_WV}});
  PerformCenterShapeTest(&engine2, "engine2 - offset box test");

  PerformCenterShapeTest(&engine1, "engine1 - second render");
}

}  // namespace
}  // namespace gl
}  // namespace render
}  // namespace geometry
}  // namespace drake
