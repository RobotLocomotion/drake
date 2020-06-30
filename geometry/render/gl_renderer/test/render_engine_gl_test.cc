#include "drake/geometry/render/gl_renderer/render_engine_gl.h"

#include <array>
#include <cstring>
#include <unordered_map>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/render_label.h"
#include "drake/systems/sensors/color_palette.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

using Eigen::Translation3d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransformd;
using std::make_unique;
using std::unique_ptr;
using std::unordered_map;
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
const double kDepthTolerance = 2.5e-4;  // meters.

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

class RenderEngineGlTest : public ::testing::Test {
 public:
  RenderEngineGlTest()
      : depth_(kWidth, kHeight),
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
              ImageDepth32F* depth_in = nullptr) {
    if (!renderer) renderer = renderer_.get();
    const DepthCameraProperties& camera = camera_in ? *camera_in : camera_;
    ImageDepth32F* depth = depth_in ? depth_in : &depth_;
    EXPECT_NO_THROW(renderer->RenderDepthImage(camera, depth));
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
                      const DepthCameraProperties& camera,
                      ImageDepth32F* depth_in = nullptr) {
    ImageDepth32F& depth = depth_in ? *depth_in : depth_;

    for (const auto& screen_coord : GetOutliers(camera)) {
      // Depth
      EXPECT_TRUE(IsExpectedDepth(depth, screen_coord, expected_outlier_depth_,
                                  kDepthTolerance));
    }
  }

  void SetUp() override {}

  // All tests on this class must invoke this first.
  void SetUp(const RigidTransformd& X_WR, bool add_terrain = false) {
    renderer_ = make_unique<RenderEngineGl>();
    renderer_->UpdateViewpoint(X_WR);

    if (add_terrain) {
      const GeometryId ground_id = GeometryId::get_new_id();
      renderer_->RegisterVisual(ground_id, HalfSpace(), PerceptionProperties(),
                                RigidTransformd::Identity(),
                                false /* needs update */);
    }
  }

  // Creates a simple perception properties set for fixed, known results.
  PerceptionProperties simple_material() const {
    PerceptionProperties material;
    Vector4d color(kDefaultVisualColor.r / 255., kDefaultVisualColor.g / 255.,
                   kDefaultVisualColor.b / 255., 1.);
    material.AddProperty("phong", "diffuse", color);
    // NOTE: Any render label is sufficient; we aren't testing them for this
    // depth-only renderer.
    material.AddProperty("label", "id", RenderLabel::kEmpty);
    return material;
  }

  // Populates the given renderer with the sphere required for
  // PerformCenterShapeTest().
  void PopulateSphereTest(RenderEngineGl* renderer) {
    Sphere sphere{0.5};
    renderer->RegisterVisual(geometry_id_, sphere, PerceptionProperties(),
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
  void PerformCenterShapeTest(RenderEngineGl* renderer,
                              const DepthCameraProperties* camera = nullptr) {
    const DepthCameraProperties& cam = camera ? *camera : camera_;
    // Can't use the member images in case the camera has been configured to a
    // different size than the default camera_ configuration.
    ImageDepth32F depth(cam.width, cam.height);
    Render(renderer, &cam, &depth);

    VerifyOutliers(*renderer, cam, &depth);

    // Verifies inside the sphere.
    const ScreenCoord inlier = GetInlier(cam);
    EXPECT_TRUE(IsExpectedDepth(depth, inlier, expected_object_depth_,
                                kDepthTolerance));
  }

  // Provide a default visual color for this tests -- it is intended to be
  // different from the default color of the VTK render engine.
  const ColorI kDefaultVisualColor = {229u, 229u, 229u};
  const float kDefaultDistance{3.f};

  // Values to be used with the "centered shape" tests.
  // The amount inset from the edge of the images to *still* expect terrain
  // values.
  static constexpr int kInset{10};
  float expected_outlier_depth_{3.f};
  float expected_object_depth_{2.f};

  const DepthCameraProperties camera_ = {kWidth, kHeight, kFovY,
                                         "n/a",  kZNear,  kZFar};
  ImageDepth32F depth_;
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

  SCOPED_TRACE("NoBodyTest");
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
    SCOPED_TRACE(fmt::format("TerrainTest: depth = {}", depth));
    VerifyUniformDepth(depth);
  }

  {
    // Closer than kZNear.
    p_WR.z() = kZNear - 1e-5;
    X_WR_.set_translation(p_WR);
    renderer_->UpdateViewpoint(X_WR_);
    Render();
    SCOPED_TRACE("TerrainTest: ground closer than z-near");
    VerifyUniformDepth(InvalidDepth::kTooClose);
  }
  {
    // Farther than kZFar.
    p_WR.z() = kZFar + 1e-3;
    X_WR_.set_translation(p_WR);
    renderer_->UpdateViewpoint(X_WR_);
    Render();
    SCOPED_TRACE("TerrainTest: ground farther than z-far");
    VerifyUniformDepth(InvalidDepth::kTooFar);
  }
}

// Performs the shape centered in the image with a box.
TEST_F(RenderEngineGlTest, BoxTest) {
  SetUp(X_WR_, true);

  // Sets up a box.
  Box box(1, 1, 1);
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, box, simple_material(),
                            RigidTransformd::Identity(),
                            true /* needs update */);
  RigidTransformd X_WV{Translation3d(0, 0, 0.5)};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});

  SCOPED_TRACE("Box test");
  PerformCenterShapeTest(renderer_.get());
}

// Performs the shape centered in the image with a sphere.
TEST_F(RenderEngineGlTest, SphereTest) {
  SetUp(X_WR_, true);

  PopulateSphereTest(renderer_.get());

  SCOPED_TRACE("Sphere test");
  PerformCenterShapeTest(renderer_.get());
}

// Performs the shape centered in the image with a cylinder.
TEST_F(RenderEngineGlTest, CylinderTest) {
  SetUp(X_WR_, true);

  // Sets up a cylinder.
  Cylinder cylinder(0.2, 1.2);
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, cylinder, simple_material(),
                            RigidTransformd::Identity(),
                            true /* needs update */);
  // Position the top of the cylinder to be 1 m above the terrain.
  RigidTransformd X_WV{Translation3d(0, 0, 0.4)};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});

  SCOPED_TRACE("Cylinder test");
  PerformCenterShapeTest(renderer_.get());
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

  SCOPED_TRACE("Mesh test");
  PerformCenterShapeTest(renderer_.get());
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
  SetUp(X_WR_, true);
  const float default_depth = expected_object_depth_;
  const double kRadius = 0.5;

  // Positions a sphere centered at <0, 0, z> with the given color.
  auto add_sphere = [this, kRadius](double z, GeometryId geometry_id) {
    const Sphere sphere{kRadius};
    const float depth = kDefaultDistance - kRadius - z;
    PerceptionProperties material;
    renderer_->RegisterVisual(geometry_id, sphere, material,
                              RigidTransformd::Identity(), true);
    const RigidTransformd X_WV{Translation3d(0, 0, z)};
    X_WV_.insert({geometry_id, X_WV});
    renderer_->UpdatePoses(X_WV_);
    return depth;
  };

  // Sets the expected values prior to calling PerformCenterShapeTest().
  auto set_expectations = [this](float depth) {
    expected_object_depth_ = depth;
  };

  {
    // Add initial sphere - desired position of the *top* of the sphere is
    // z₀ = 1.0.
    const GeometryId id = GeometryId::get_new_id();
    const float depth = add_sphere(1.0 - kRadius, id);
    ASSERT_EQ(depth, default_depth);
    set_expectations(depth);
    SCOPED_TRACE("First sphere test");
    PerformCenterShapeTest(renderer_.get());
  }

  // We'll be adding and removing two spheres; get their ids ready.
  const GeometryId id1 = GeometryId::get_new_id();
  const GeometryId id2 = GeometryId::get_new_id();

  {
    // Add another sphere of a different color in front of the default sphere.
    // Desired position of the *top* of the sphere is z₁ = 1.5.
    const float depth = add_sphere(1.5 - kRadius, id1);
    set_expectations(depth);
    SCOPED_TRACE("Second sphere added in remove test");
    PerformCenterShapeTest(renderer_.get());
  }

  {
    // Add a _third_ sphere in front of the second.
    // Desired position of the *top* of the sphere is z₂ = 2.0.
    float depth = add_sphere(2.0 - kRadius, id2);
    set_expectations(depth);
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
    set_expectations(default_depth);
    SCOPED_TRACE("Default image restored by removing extra geometries");
    PerformCenterShapeTest(renderer_.get());
  }
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
  SCOPED_TRACE("Simple clone");
  PerformCenterShapeTest(dynamic_cast<RenderEngineGl*>(clone.get()));
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
  SCOPED_TRACE("Clone persistence");
  PerformCenterShapeTest(static_cast<RenderEngineGl*>(clone.get()));
}

// Tests that the cloned renderer still works, even when the original has values
// changed.
TEST_F(RenderEngineGlTest, CloneIndependence) {
  SetUp(X_WR_, true);
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
  SetUp(X_WR_, true);
  PopulateSphereTest(renderer_.get());

  {
    // Baseline -- confirm that all of the defaults in this test still produce
    // the expected outcome.
    SCOPED_TRACE("Camera change - baseline");
    PerformCenterShapeTest(renderer_.get(), &camera_);
  }

  // Test changes in sensor sizes.
  {
    // Now run it again with a camera with a smaller sensor (quarter area).
    DepthCameraProperties small_camera{camera_};
    small_camera.width /= 2;
    small_camera.height /= 2;
    {
      SCOPED_TRACE("Camera change - small camera");
      PerformCenterShapeTest(renderer_.get(), &small_camera);
    }

    // Now run it again with a camera with a bigger sensor (4X area).
    {
      DepthCameraProperties big_camera{camera_};
      big_camera.width *= 2;
      big_camera.height *= 2;
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
    DepthCameraProperties narrow_fov(camera_);
    narrow_fov.fov_y /= 2;
    SCOPED_TRACE("Camera change - narrow fov");
    PerformCenterShapeTest(renderer_.get(), &narrow_fov);
  }

  {
    DepthCameraProperties wide_fov(camera_);
    wide_fov.fov_y *= 2;
    SCOPED_TRACE("Camera change - wide fov");
    PerformCenterShapeTest(renderer_.get(), &wide_fov);
  }

  // Test changes to depth range.
  {
    DepthCameraProperties clipping_far_plane(camera_);
    clipping_far_plane.z_far = expected_outlier_depth_ - 0.1;
    const float old_outlier_depth = expected_outlier_depth_;
    expected_outlier_depth_ = std::numeric_limits<float>::infinity();
    {
      SCOPED_TRACE("Camera change - z far clips terrain");
      PerformCenterShapeTest(renderer_.get(), &clipping_far_plane);
      // NOTE: Need to restored expected outlier depth for next test.
      expected_outlier_depth_ = old_outlier_depth;
    }

    {
      DepthCameraProperties clipping_near_plane(camera_);
      clipping_near_plane.z_near = expected_object_depth_ + 0.1;
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
  SetUp(X_WR_, false /* no terrain */);

  // Sets up a box.
  Box box(1, 1, 1);
  const GeometryId id = GeometryId::get_new_id();
  renderer_->RegisterVisual(id, box, PerceptionProperties(),
                            RigidTransformd::Identity(), true);
  RigidTransformd X_WV{Translation3d(0, 0, 0.5)};
  renderer_->UpdatePoses(
      unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});

  EXPECT_NO_THROW(Render());
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
    renderer->RegisterVisual(ground, HalfSpace(), PerceptionProperties(),
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

TEST_F(RenderEngineGlTest, RenderColorImageThrows) {
  RenderEngineGl engine;
  CameraProperties camera{2, 2, M_PI, "junk"};
  ImageRgba8U image{camera.width, camera.height};
  DRAKE_EXPECT_THROWS_MESSAGE(engine.RenderColorImage(camera, false, &image),
                              std::runtime_error,
                              "RenderEngineGl cannot render color images");
}

TEST_F(RenderEngineGlTest, RenderLabelImageThrows) {
  RenderEngineGl engine;
  CameraProperties camera{2, 2, M_PI, "junk"};
  ImageLabel16I image{camera.width, camera.height};
  DRAKE_EXPECT_THROWS_MESSAGE(engine.RenderLabelImage(camera, false, &image),
                              std::runtime_error,
                              "RenderEngineGl cannot render label images");
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
