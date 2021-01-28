#include "drake/geometry/render/render_engine.h"

#include <optional>
#include <set>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/test_utilities/dummy_render_engine.h"

namespace drake {
namespace geometry {
namespace render {

class RenderEngineTester {
 public:
  explicit RenderEngineTester(const RenderEngine* engine) : engine_(*engine) {}

  int num_geometries() const {
    return static_cast<int>(engine_.update_ids_.size() +
                            engine_.anchored_ids_.size());
  }

  bool has_id(GeometryId id) const {
    return engine_.update_ids_.count(id) > 0 ||
           engine_.anchored_ids_.count(id) > 0;
  }

 private:
  const RenderEngine& engine_;
};

namespace {

using Eigen::Vector3d;
using geometry::internal::DummyRenderEngine;
using math::RigidTransformd;
using std::set;
using std::unordered_map;
using systems::sensors::CameraInfo;
using systems::sensors::ColorD;
using systems::sensors::ColorI;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

// This is the absolutely minimum code to make an instantiable RenderEngine.
// It is *not* an example appropriate implementation of RenderEngine, it is
// merely a class of convenience for these tests so that individual aspects
// can be implemented in a targeted manner without worrying about this baseline
// functionality.
// This mainline implementation:
//    - UpdateViewpoint and DoUpdateVisualPose are no-ops.
//    - DoRegisterVisual and DoRemoveGeometry do nothing and report such.
//    - DoClone simply returns nullptr.
//    - Implements no rendering interfaces; invocations of
//      MinimumEngine::Render*Image (in any variant) will throw.
class MinimumEngine : public RenderEngine {
 public:
  void UpdateViewpoint(const math::RigidTransformd&) override {}
  bool DoRegisterVisual(GeometryId, const Shape&, const PerceptionProperties&,
                        const RigidTransformd&) override {
    return false;
  }
  void DoUpdateVisualPose(GeometryId, const math::RigidTransformd&) override {}
  bool DoRemoveGeometry(GeometryId) override { return false; }
  std::unique_ptr<RenderEngine> DoClone() const override { return nullptr; }
};

// Tests the RenderEngine-specific functionality for managing registration of
// geometry and its corresponding update behavior. The former should configure
// each geometry correctly on whether it gets updated or not, and the latter
// will confirm that the right geometries get updated.
GTEST_TEST(RenderEngine, RegistrationAndUpdate) {
  // Change the default render label to something registerable.
  DummyRenderEngine engine({RenderLabel::kDontCare});

  // Configure parameters for registering visuals.
  PerceptionProperties skip_properties = engine.rejecting_properties();
  PerceptionProperties add_properties = engine.accepting_properties();
  Sphere sphere(1.0);

  // A collection of poses to provide to calls to UpdatePoses(). Configured
  // to all identity transforms because the values generally don't matter. In
  // the single case where it does matter, a value is explicitly set (see
  // below).
  unordered_map<GeometryId, RigidTransformd> X_WG_all{
      {GeometryId::get_new_id(), RigidTransformd{Vector3d{1, 2, 3}}},
      {GeometryId::get_new_id(), RigidTransformd{Vector3d{2, 3, 4}}},
      {GeometryId::get_new_id(), RigidTransformd{Vector3d{3, 4, 5}}}};

  // These test cases are accumulative; re-ordering them will require
  // refactoring.

  // Tests that rely on the RenderEngine's default value are tested below in the
  // DefaultRenderLabel test.

  // Create properties with the given RenderLabel value. Used only to test
  // failure when providing an invalid render label. (See Case 1.)
  auto make_properties = [](RenderLabel label) {
    PerceptionProperties properties;
    properties.AddProperty("label", "id", label);
    return properties;
  };

  // Case 1: Explicitly providing the unspecified or empty render label throws.
  for (const auto& label : {RenderLabel::kEmpty, RenderLabel::kUnspecified}) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        engine.RegisterVisual(GeometryId::get_new_id(), sphere,
                              make_properties(label), {},
                              false),
        std::logic_error,
        "Cannot register a geometry with the 'unspecified' or 'empty' render "
        "labels.*");
  }

  // Case: the shape is configured to be ignored by the render engine. Returns
  // false (and other arguments do not matter).
  const GeometryId id1 = GeometryId::get_new_id();
  const bool is_dynamic = true;
  const bool dynamic_accepted =
      engine.RegisterVisual(id1, sphere, skip_properties, {}, is_dynamic);
  EXPECT_FALSE(dynamic_accepted);
  const bool anchored_accepted =
      engine.RegisterVisual(id1, sphere, skip_properties, {}, !is_dynamic);
  EXPECT_FALSE(anchored_accepted);
  // Confirm nothing is updated - because nothing is registered.
  engine.UpdatePoses(X_WG_all);
  EXPECT_EQ(engine.updated_ids().size(), 0);

  {
    // Case: the shape is configured for registration, but does *not* require
    // updating.
    const auto& [id, X_WG] = *(X_WG_all.begin());
    bool accepted = engine.RegisterVisual(id, sphere,
                                          add_properties, X_WG, !is_dynamic);
    EXPECT_TRUE(accepted);
    EXPECT_TRUE(CompareMatrices(engine.world_pose(id).GetAsMatrix34(),
                                X_WG.GetAsMatrix34()));
    engine.UpdatePoses(X_WG_all);
    EXPECT_EQ(engine.updated_ids().size(), 0);
    EXPECT_TRUE(CompareMatrices(engine.world_pose(id).GetAsMatrix34(),
                                X_WG.GetAsMatrix34()));
  }

  {
    // Case: the shape is configured for registration *and* requires updating.
    // Configure the pose for the id so it is *not* the identity.
    const auto& [id, X_WG] = *(++(X_WG_all.begin()));
    bool accepted =
        engine.RegisterVisual(id, sphere, add_properties, X_WG, is_dynamic);
    EXPECT_TRUE(accepted);
    const Vector3d p_WG(1.5, 2.5, 3.5);
    X_WG_all[id].set_translation(p_WG);
    engine.UpdatePoses(X_WG_all);
    EXPECT_EQ(engine.updated_ids().size(), 1);
    ASSERT_EQ(engine.updated_ids().count(id), 1);
    EXPECT_TRUE(
        CompareMatrices(engine.updated_ids().at(id).translation(), p_WG));
    EXPECT_TRUE(CompareMatrices(engine.world_pose(id).GetAsMatrix34(),
                                X_WG_all[id].GetAsMatrix34()));
  }
}

// Tests the removal of geometry from the renderer -- confirms that the
// RenderEngine removes the geometry appropriately.
GTEST_TEST(RenderEngine, RemoveGeometry) {
  const int need_update_count = 3;
  const int anchored_count = 2;

  // The test render engine contains three dynamic geometries and two anchored.

  // Set the default render label to something registerable.
  DummyRenderEngine engine({RenderLabel::kDontCare});
  // A set of properties that will cause a shape to be properly registered.
  PerceptionProperties add_properties = engine.accepting_properties();
  RigidTransformd X_WG = RigidTransformd::Identity();
  Sphere sphere(1.0);

  set<GeometryId> ids;

  for (int i = 0; i < need_update_count + anchored_count; ++i) {
    const GeometryId id = GeometryId::get_new_id();
    const bool is_dynamic = i % 2 == 0;  // alternate dynamic, anchored, etc.
    const bool accepted = engine.RegisterVisual(id, sphere, add_properties,
                                                X_WG, is_dynamic);
    if (!accepted) {
      throw std::logic_error("The geometry wasn't accepted for registration");
    }
    ids.insert(id);
  }
  RenderEngineTester tester(&engine);

  // Case: invalid ids don't get removed.
  EXPECT_FALSE(engine.RemoveGeometry(GeometryId::get_new_id()));

  // Case: Systematically remove remaining geometries and confirm state --
  // because of how geometries were added, this will alternate dynamic,
  // anchored, dynamic, etc.
  while (!ids.empty()) {
    const GeometryId id = *ids.begin();
    EXPECT_TRUE(engine.RemoveGeometry(id));
    ids.erase(ids.begin());
    EXPECT_EQ(static_cast<int>(ids.size()), tester.num_geometries());
    for (GeometryId remaining_id : ids) {
      EXPECT_TRUE(tester.has_id(remaining_id));
    }
  }
}

GTEST_TEST(RenderEngine, ColorLabelConversion) {
  // Explicitly testing labels at *both* ends of the reserved space -- this
  // assumes that the reserved labels are at the top end; if that changes, we'll
  // need a different mechanism to get a large-valued label.
  RenderLabel label1 = RenderLabel(0);
  RenderLabel label2 = RenderLabel(RenderLabel::kMaxUnreserved - 1);
  RenderLabel label3 = RenderLabel::kEmpty;

  // A ColorI should be invertible back to the original label.
  ColorI color1 = DummyRenderEngine::GetColorIFromLabel(label1);
  ColorI color2 = DummyRenderEngine::GetColorIFromLabel(label2);
  ColorI color3 = DummyRenderEngine::GetColorIFromLabel(label3);
  EXPECT_EQ(label1, DummyRenderEngine::LabelFromColor(color1));
  EXPECT_EQ(label2, DummyRenderEngine::LabelFromColor(color2));
  EXPECT_EQ(label3, DummyRenderEngine::LabelFromColor(color3));

  // Different labels should produce different colors.
  ASSERT_NE(label1, label2);
  ASSERT_NE(label2, label3);
  ASSERT_NE(label1, label3);
  auto same_colors = [](const auto& expected, const auto& test) {
    if (expected.r != test.r || expected.g != test.g || expected.b != test.b) {
      return ::testing::AssertionFailure()
          << "Expected color " << expected << ", found " << test;
    }
    return ::testing::AssertionSuccess();
  };

  EXPECT_FALSE(same_colors(color1, color2));
  EXPECT_FALSE(same_colors(color2, color3));
  EXPECT_FALSE(same_colors(color1, color3));

  // Different labels should also produce different Normalized colors.
  ColorD color1_d = DummyRenderEngine::GetColorDFromLabel(label1);
  ColorD color2_d = DummyRenderEngine::GetColorDFromLabel(label2);
  ColorD color3_d = DummyRenderEngine::GetColorDFromLabel(label3);
  EXPECT_FALSE(same_colors(color1_d, color2_d));
  EXPECT_FALSE(same_colors(color1_d, color3_d));
  EXPECT_FALSE(same_colors(color2_d, color3_d));

  // The normalized color should simply be the integer color divided by 255.
  ColorD color1_d_by_hand{color1.r / 255., color1.g / 255., color1.b / 255.};
  ColorD color2_d_by_hand{color2.r / 255., color2.g / 255., color2.b / 255.};
  ColorD color3_d_by_hand{color3.r / 255., color3.g / 255., color3.b / 255.};

  EXPECT_TRUE(same_colors(color1_d, color1_d_by_hand));
  EXPECT_TRUE(same_colors(color2_d, color2_d_by_hand));
  EXPECT_TRUE(same_colors(color3_d, color3_d_by_hand));
}

// Tests the documented behavior for configuring the default render label.
GTEST_TEST(RenderEngine, DefaultRenderLabel) {
  // Case: Confirm RenderEngine default is kUnspecified.
  {
    DummyRenderEngine engine;
    EXPECT_EQ(engine.default_render_label(), RenderLabel::kUnspecified);
  }

  // Case: Confirm kDontCare is valid.
  {
    DummyRenderEngine engine(RenderLabel::kDontCare);
    EXPECT_EQ(engine.default_render_label(), RenderLabel::kDontCare);
  }

  // Case: Confirm construction with alternate label is forbidden.
  {
    for (auto label :
         {RenderLabel::kDoNotRender, RenderLabel::kEmpty, RenderLabel{10}}) {
      DRAKE_EXPECT_THROWS_MESSAGE(DummyRenderEngine{label}, std::logic_error,
                                  ".* default render label must be either "
                                  "'kUnspecified' or 'kDontCare'");
    }
  }
}

// The render API with full intrinsics promises some validation before calling
// the virtual DoRender*Image() API. Confirm that it happens.
GTEST_TEST(RenderEngine, ValidateIntrinsicsAndImage) {
  const DummyRenderEngine engine;
  const int w = 2;
  const int h = 2;
  const CameraInfo intrinsics{w, h, M_PI};
  const ColorRenderCamera color_camera{
      {"n/a", intrinsics, {0.1, 10}, RigidTransformd{}}, false};
  const DepthRenderCamera depth_camera{
      {"n/a", intrinsics, {0.1, 10}, RigidTransformd{}}, {1.0, 5.0}};

  // Image is nullptr.
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.RenderColorImage(color_camera, nullptr),
      std::logic_error,
      "Can't render a color image. The given output image is nullptr");
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.RenderDepthImage(depth_camera, nullptr),
      std::logic_error,
      "Can't render a depth image. The given output image is nullptr");
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.RenderLabelImage(color_camera, nullptr),
      std::logic_error,
      "Can't render a label image. The given output image is nullptr");

  // Image size doesn't match.
  const char* error_message =
      "The {} image to write has a size different from that specified in "
      "the camera intrinsics. Image: \\({}, {}\\), intrinsics: \\({}, {}\\)";

  auto test_bad_sizes = [&](const char* image_type, auto image_example,
                            auto render) {
    using Image = decltype(image_example);
    std::vector<Image> images{Image{w - 1, h}, Image{w, h - 1},
                              Image{w - 1, h - 1}};
    for (Image& i : images) {
      DRAKE_EXPECT_THROWS_MESSAGE(
          render(&i), std::logic_error,
          fmt::format(error_message, image_type, i.width(), i.height(),
                      intrinsics.width(), intrinsics.height()));
    }
  };

  test_bad_sizes("color", ImageRgba8U{1, 1}, [&](ImageRgba8U* i) {
    engine.RenderColorImage(color_camera, i);
  });

  test_bad_sizes("depth", ImageDepth32F{1, 1}, [&](ImageDepth32F* i) {
    engine.RenderDepthImage(depth_camera, i);
  });

  test_bad_sizes("label", ImageLabel16I{1, 1}, [&](ImageLabel16I* i) {
    engine.RenderLabelImage(color_camera, i);
  });
}

// An absolute barebones RenderEngine implementation; however it is cloneable
// with both a copy constructor *and* a valid DoClone() implementation.
class CloneableEngine : public MinimumEngine {
 public:
  CloneableEngine(const CloneableEngine&) = default;
  CloneableEngine& operator=(const CloneableEngine&) = delete;
  CloneableEngine(CloneableEngine&&) = delete;
  CloneableEngine& operator=(CloneableEngine&&) = delete;

  CloneableEngine() = default;

  std::unique_ptr<RenderEngine> DoClone() const override {
    return std::make_unique<CloneableEngine>(*this);
  }
};

// A derivative of CloneableEngine that has forgotten to implement DoClone().
// That should produce an error.
class NoDoCloneEngine : public CloneableEngine {
 public:
  NoDoCloneEngine() = default;
};

// Confirms that the guard to detect a derived RenderEngine that forgot to
// implement DoClone().
GTEST_TEST(RenderEngine, DetectDoCloneFailure) {
  CloneableEngine cloneable;
  EXPECT_NO_THROW(cloneable.Clone());

  NoDoCloneEngine not_cloneable;
  DRAKE_EXPECT_THROWS_MESSAGE(
      not_cloneable.Clone(), std::logic_error,
      "Error in cloning RenderEngine class of type .+NoDoCloneEngine; the "
      "clone returns type .+CloneableEngine. .+NoDoCloneEngine::DoClone.. was "
      "probably not implemented");
}

// Test infrastructure to support deprecation efforts. After completing the
// deprecation (2021-04-01), all of this test infrastructure can/should simply
// go away. In the deprecation period there are three possible configurations of
// a child RenderEngine class:
//
//   - Failed to implement either the old or new APIs.
//     - This should be detected and an error thrown. See
//       the test DetectUnimplementedRender below.
//   - Implementation of new API (invocation of old delegates to new).
//     - The DummyRenderEngine satisfies this case and allows detection of
//       delegation. See the test OldApiDelegatesToNewApi
//       below.
//   - Implementaiton of old API (invocation of new delegates to old).
//     - See the test NewApiDelegatesToOldApi.
//
// There is *technically* a fourth option, implementation of both old and new
// APIs. If a subclass does that, then the RenderEngine API doesn't participate
// at all and that scenario is outside the scope of the tests for RenderEngine.
//
// https://github.com/RobotLocomotion/drake/issues/11880

// The desired behavior in the deprecation period is for the user to implement
// DoRender*Image and calls to RenderImage(CamProperties) get automatically
// forwarded. DummyRenderEngine has this expected implementation. Confirm
// the forwarding. To that end, we'll confirm two things: that the DoRender*
// function got called and that the camera provided is what we expect.
GTEST_TEST(RenderEngine, OldApiDelegatesToNewApi) {
  // Arbitrary image sizes that are unlikely to be hard-coded anywhere.
  const int w = 35;
  const int h = 45;

  ImageRgba8U color(w, h);
  ImageDepth32F depth(w, h);
  ImageLabel16I label(w, h);

  // DummyRenderEngine implements the new DoRender*Image API and *not* the old
  // CameraProperties-based API.
  DummyRenderEngine engine;

  auto cores_equal = [](const RenderCameraCore& test,
                        const RenderCameraCore& expected) {
    return test.renderer_name() == expected.renderer_name() &&
           test.clipping().near() == expected.clipping().near() &&
           test.clipping().far() == expected.clipping().far() &&
           CompareMatrices(
               test.sensor_pose_in_camera_body().GetAsMatrix34(),
               expected.sensor_pose_in_camera_body().GetAsMatrix34()) &&
           CompareMatrices(test.intrinsics().intrinsic_matrix(),
                           expected.intrinsics().intrinsic_matrix());
  };

  auto color_cameras_equal = [&cores_equal](const ColorRenderCamera& test,
                                            const ColorRenderCamera& expected) {
    return test.show_window() == expected.show_window() &&
           cores_equal(test.core(), expected.core());
  };

  auto depth_cameras_equal = [&cores_equal](const DepthRenderCamera& test,
                                            const DepthRenderCamera& expected) {
    return test.depth_range().min_depth() ==
               expected.depth_range().min_depth() &&
           test.depth_range().max_depth() ==
               expected.depth_range().max_depth() &&
           cores_equal(test.core(), expected.core());
  };

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const DepthCameraProperties properties{w, h, M_PI / 3, "n/a", 0.25, 11.5};

  engine.RenderColorImage(properties, true, &color);
  EXPECT_EQ(engine.num_color_renders(), 1);
  EXPECT_TRUE(color_cameras_equal(engine.last_color_camera(),
                                  ColorRenderCamera(properties, true)));

  engine.RenderColorImage(properties, false, &color);
  EXPECT_EQ(engine.num_color_renders(), 2);
  EXPECT_TRUE(color_cameras_equal(engine.last_color_camera(),
                                  ColorRenderCamera(properties, false)));

  engine.RenderDepthImage(properties, &depth);
  EXPECT_EQ(engine.num_depth_renders(), 1);
  EXPECT_TRUE(depth_cameras_equal(engine.last_depth_camera(),
                                  DepthRenderCamera(properties)));

  engine.RenderLabelImage(properties, true, &label);
  EXPECT_EQ(engine.num_label_renders(), 1);
  EXPECT_TRUE(color_cameras_equal(engine.last_label_camera(),
                                  ColorRenderCamera(properties, true)));

  engine.RenderLabelImage(properties, false, &label);
  EXPECT_EQ(engine.num_label_renders(), 2);
  EXPECT_TRUE(color_cameras_equal(engine.last_label_camera(),
                                  ColorRenderCamera(properties, false)));
#pragma GCC diagnostic pop
}

// Confirms that a RenderEngine that doesn't implement any rendering API is
// detected.
GTEST_TEST(RenderEngine, DetectUnimplementedRender) {
  // Set up cameras and images.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const CameraProperties cam_props(10, 10, M_PI / 4, "n/a");
  const DepthCameraProperties depth_props(cam_props.width, cam_props.height,
                                          cam_props.fov_y,
                                          cam_props.renderer_name, 0.2, 8.0);
#pragma GCC diagnostic pop
  const ColorRenderCamera color_cam(cam_props);
  const DepthRenderCamera depth_cam(depth_props);
  ImageRgba8U color_image(cam_props.width, cam_props.height);
  ImageDepth32F depth_image(depth_props.width, depth_props.height);
  ImageLabel16I label_image(cam_props.width, cam_props.height);

  // Remember, MinimumEngine has implemented no rendering logic.
  MinimumEngine engine;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.RenderColorImage(cam_props, false, &color_image), std::exception,
      ".+render a color image.+RenderColorImage.+DoRenderColorImage.+");
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.RenderDepthImage(depth_props, &depth_image), std::exception,
      ".+render a depth image.+RenderDepthImage.+DoRenderDepthImage.+");
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.RenderLabelImage(cam_props, false, &label_image), std::exception,
      ".+render a label image.+RenderLabelImage.+DoRenderLabelImage.+");
#pragma GCC diagnostic pop

  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.RenderColorImage(color_cam, &color_image), std::exception,
      ".+render a color image.+RenderColorImage.+DoRenderColorImage.+");
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.RenderDepthImage(depth_cam, &depth_image), std::exception,
      ".+render a depth image.+RenderDepthImage.+DoRenderDepthImage.+");
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.RenderLabelImage(color_cam, &label_image), std::exception,
      ".+render a label image.+RenderLabelImage.+DoRenderLabelImage.+");
}


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

// This class implements *only* the legacy API to validate forwarding of APIs.
// Calls on the old API are counted, and the camera properties are cached for
// later examination.
class LegacyEngine final : public MinimumEngine {
 public:
  LegacyEngine()
      : color_props_{0, 0, 0, ""},
        depth_props_{0, 0, 0, "", 0, 0},
        label_props_{0, 0, 0, ""} {}

  // Make sure the new API is available (even though we're not implementing it).
  using MinimumEngine::RenderColorImage;
  using MinimumEngine::RenderDepthImage;
  using MinimumEngine::RenderLabelImage;

  void RenderColorImage(const CameraProperties& camera, bool,
                        ImageRgba8U*) const final {
    ++color_count_;
    color_props_ = camera;
  }
  void RenderDepthImage(const DepthCameraProperties& camera,
                        ImageDepth32F*) const final {
    ++depth_count_;
    depth_props_ = camera;
  }
  void RenderLabelImage(const CameraProperties& camera, bool,
                        ImageLabel16I*) const final {
    ++label_count_;
    label_props_ = camera;
  }

  int num_color_renders() const { return color_count_; }
  int num_depth_renders() const { return depth_count_; }
  int num_label_renders() const { return label_count_; }

  const CameraProperties& last_color_camera_properties() const {
    return color_props_;
  }
  const DepthCameraProperties& last_depth_camera_properties() const {
    return depth_props_;
  }
  const CameraProperties& last_label_camera_properties() const {
    return label_props_;
  }

 private:
  mutable int color_count_{};
  mutable int depth_count_{};
  mutable int label_count_{};
  mutable CameraProperties color_props_;
  mutable DepthCameraProperties depth_props_;
  mutable CameraProperties label_props_;
};

// Tests the case where a RenderEngine implements the old API, but not the new
// API. Confirms that the delegation occurs and the camera is converted as
// expected.
GTEST_TEST(RenderEngine, NewApiDelegatesToOldApi) {
  const CameraInfo intrinsics{2, 2, M_PI};
  const ColorRenderCamera color_camera{
      {"n/a", intrinsics, {0.1, 10}, RigidTransformd{}}, false};
  const DepthRenderCamera depth_camera{
      {"n/a", intrinsics, {0.1, 10}, RigidTransformd{}}, {1.0, 5.0}};

  ImageRgba8U color(2, 2);
  ImageDepth32F depth(2, 2);
  ImageLabel16I label(2, 2);

  LegacyEngine engine;

  // Confirm all counters are as expected before starting.
  ASSERT_EQ(engine.num_color_renders(), 0);
  ASSERT_EQ(engine.num_depth_renders(), 0);
  ASSERT_EQ(engine.num_label_renders(), 0);

  auto cameras_match = [](const CameraProperties& test,
                          const RenderCameraCore& expected) {
    return test.width == expected.intrinsics().width() &&
           test.height == expected.intrinsics().height() &&
           test.fov_y == expected.intrinsics().fov_y() &&
           test.renderer_name == expected.renderer_name();
  };

  auto depths_match = [](const DepthCameraProperties& test,
                         const DepthRenderCamera& expected) {
    return test.z_near == expected.depth_range().min_depth() &&
           test.z_far == expected.depth_range().max_depth();
  };

  engine.RenderColorImage(color_camera, &color);
  EXPECT_EQ(engine.num_color_renders(), 1);
  EXPECT_TRUE(cameras_match(engine.last_color_camera_properties(),
                            color_camera.core()));
  engine.RenderDepthImage(depth_camera, &depth);
  EXPECT_EQ(engine.num_depth_renders(), 1);
  EXPECT_TRUE(cameras_match(engine.last_depth_camera_properties(),
                            depth_camera.core()));
  EXPECT_TRUE(
      depths_match(engine.last_depth_camera_properties(), depth_camera));
  engine.RenderLabelImage(color_camera, &label);
  EXPECT_EQ(engine.num_label_renders(), 1);
  EXPECT_TRUE(cameras_match(engine.last_label_camera_properties(),
                            color_camera.core()));
}
#pragma GCC diagnostic pop

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
