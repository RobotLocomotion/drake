#include "drake/systems/sensors/rgbd_sensor.h"

#include <functional>
#include <memory>
#include <type_traits>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_state.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/test_utilities/dummy_render_engine.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace geometry {

// Friend class provides access to the render engine in the GeometryState. This
// is important because we need access to the render engine in the *context*
// which is a copy of the engine that was initially instantiated and registered
// with SceneGraph.
template <class T>
class GeometryStateTester {
 public:
  // Extract a dummy render engine from the given `context` with the given
  // `name`. Blindly assumes the context belongs to a SceneGraph and that
  // the renderer with the given name *is* a DummyRenderEngine. (If no renderer
  // with that name exists, it will simply throw.)
  static const internal::DummyRenderEngine& GetDummyRenderEngine(
      const systems::Context<T>& context, const std::string& name) {
    // Technically brittle, but relatively safe assumption that GeometryState
    // is abstract Parameter value 0.
    auto& geo_state = context.get_parameters()
                          .template get_abstract_parameter<GeometryState<T>>(0);
    const render::RenderEngine& engine = geo_state.GetRenderEngineOrThrow(name);
    return dynamic_cast<const internal::DummyRenderEngine&>(engine);
  }
};

}  // namespace geometry

namespace systems {
namespace sensors {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryStateTester;
using geometry::QueryObject;
using geometry::SceneGraph;
using geometry::SourceId;
using geometry::internal::DummyRenderEngine;
using geometry::render::ClippingRange;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRange;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderCameraCore;
using geometry::render::RenderEngine;
using math::RigidTransformd;
using math::RollPitchYawd;
using std::make_pair;
using std::make_unique;
using std::move;
using std::unique_ptr;
using std::vector;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;

std::ostream& operator<<(std::ostream& out, const CameraInfo& info) {
  out << "\n  width: " << info.width()
      << "\n  height: " << info.height()
      << "\n  focal_x: " << info.focal_x()
      << "\n  focal_y: " << info.focal_y()
      << "\n  center_x: " << info.center_x()
      << "\n  center_y: " << info.center_y();
  return out;
}
std::ostream& operator<<(std::ostream& out, const ColorRenderCamera& camera) {
  out << "ColorRenderCamera\n"
      << camera.core().intrinsics()
      << "\n  show_window: " << camera.show_window();
  return out;
}

std::ostream& operator<<(std::ostream& out, const DepthRenderCamera& camera) {
  out << "DepthRenderCamera\n" << camera.core().intrinsics()
      << "\n  min_depth: " << camera.depth_range().min_depth()
      << "\n  max_depth: " << camera.depth_range().max_depth();
  return out;
}

// Helper class for exercising private methods in RgbdSensor.
class RgbdSensorTester {
 public:
  static void ConvertDepth32FTo16U(const ImageDepth32F& d32,
                                   ImageDepth16U* d16) {
    RgbdSensor::ConvertDepth32FTo16U(d32, d16);
  }
};

namespace {

::testing::AssertionResult CompareCameraInfo(const CameraInfo& test,
                                             const CameraInfo& expected) {
  if (test.width() != expected.width() || test.height() != expected.height() ||
      test.focal_x() != expected.focal_x() ||
      test.focal_y() != expected.focal_y() ||
      test.center_x() != expected.center_x() ||
      test.center_y() != expected.center_y()) {
    return ::testing::AssertionFailure()
           << "Intrinsic values don't match.\n Expected " << expected
           << "\n got: " << test;
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult CompareClipping(const ClippingRange& test,
                                           const ClippingRange& expected) {
  if (test.near() != expected.near()) {
    return ::testing::AssertionFailure()
           << "Near clipping planes don't match.\n Expected " << expected.near()
           << "\n got " << test.near();
  }
  if (test.far() != expected.far()) {
    return ::testing::AssertionFailure()
           << "Far clipping planes don't match.\n Expected " << expected.far()
           << "\n got " << test.far();
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult CompareDepthRange(const DepthRange& test,
                                             const DepthRange& expected) {
  if (test.min_depth() != expected.min_depth()) {
    return ::testing::AssertionFailure()
           << "Minimum depth doesn't match.\n Expected " << expected.min_depth()
           << "\n got " << test.min_depth();
  }
  if (test.max_depth() != expected.max_depth()) {
    return ::testing::AssertionFailure()
           << "Maximum depth doesn't match.\n Expected " << expected.max_depth()
           << "\n got " << test.max_depth();
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult CompareCameraCore(const RenderCameraCore& test,
                                             const RenderCameraCore& expected) {
  ::testing::AssertionResult result{true};

  result = CompareCameraInfo(test.intrinsics(), expected.intrinsics());
  if (!result) return result;

  if (test.renderer_name() != expected.renderer_name()) {
    return ::testing::AssertionFailure()
           << "Renderer name doesn't match.\n Expected "
           << expected.renderer_name() << "\n got " << test.renderer_name();
  }

  result = CompareClipping(test.clipping(), expected.clipping());
  if (!result) return result;

  return CompareMatrices(test.sensor_pose_in_camera_body().GetAsMatrix4(),
                         expected.sensor_pose_in_camera_body().GetAsMatrix4());
}

::testing::AssertionResult Compare(const ColorRenderCamera& test,
                                   const ColorRenderCamera& expected) {
  if (test.show_window() != expected.show_window()) {
    return ::testing::AssertionFailure()
           << "'show_window' doesn't match.\n Expected "
           << expected.show_window() << "\n got " << test.show_window();
  }
  return CompareCameraCore(test.core(), expected.core());
}

::testing::AssertionResult Compare(const DepthRenderCamera& test,
                                   const DepthRenderCamera& expected) {
  auto result = CompareCameraCore(test.core(), expected.core());
  if (!result) return result;

  return CompareDepthRange(test.depth_range(), expected.depth_range());
}

class RgbdSensorTest : public ::testing::Test {
 public:
  RgbdSensorTest()
      : ::testing::Test(),
        // N.B. This is using arbitrary yet different intrinsics for color vs.
        // depth.
        color_camera_({kRendererName, {640, 480, M_PI / 4}, {0.1, 10.0}, {}},
                      false),
        depth_camera_({kRendererName, {320, 240, M_PI / 6}, {0.1, 10.0}, {}},
                      {0.1, 10}) {}

 protected:
  // Creates a Diagram with a SceneGraph and RgbdSensor connected appropriately.
  // Various components are stored in members for easy access. This should only
  // be called once per test.
  // make_sensor is a callback that will create the sensor. It is provided a
  // pointer to the SceneGraph so it has the opportunity to modify the
  // SceneGraph as it needs.
  void MakeCameraDiagram(
      std::function<unique_ptr<RgbdSensor>(SceneGraph<double>*)> make_sensor) {
    ASSERT_EQ(scene_graph_, nullptr)
        << "Only call MakeCameraDiagram() once per test";
    DiagramBuilder<double> builder;
    scene_graph_ = builder.AddSystem<SceneGraph<double>>();
    scene_graph_->AddRenderer(kRendererName, make_unique<DummyRenderEngine>());
    sensor_ = builder.AddSystem(make_sensor(scene_graph_));
    builder.Connect(scene_graph_->get_query_output_port(),
                    sensor_->query_object_input_port());
    diagram_ = builder.Build();
    context_ = diagram_->CreateDefaultContext();
    context_->DisableCaching();
    scene_graph_context_ =
        &diagram_->GetMutableSubsystemContext(*scene_graph_, context_.get());
    sensor_context_ =
        &diagram_->GetMutableSubsystemContext(*sensor_, context_.get());
    // Must get the render engine instance from the context itself.
    render_engine_ = &GeometryStateTester<double>::GetDummyRenderEngine(
        *scene_graph_context_, kRendererName);
  }

  // Confirms that the member sensor_ matches the expected properties. Part
  // of this confirmation entails rendering the camera which *may* pull on
  // an input port. The optional `pre_render_callback` should do any work
  // necessary to make the input port viable.
  ::testing::AssertionResult ValidateConstruction(
      FrameId parent_id, const RigidTransformd& X_WC_expected,
      std::function<void()> pre_render_callback = {}) const {
    if (sensor_->parent_frame_id() != parent_id) {
      return ::testing::AssertionFailure()
             << "The sensor's parent id (" << sensor_->parent_frame_id()
             << ") does not match the expected id (" << parent_id << ")";
    }
    ::testing::AssertionResult result = ::testing::AssertionSuccess();
    result = CompareCameraInfo(
        sensor_->color_camera_info(), color_camera_.core().intrinsics());
    if (!result) return result;

    result = Compare(sensor_->color_render_camera(), color_camera_);
    if (!result) return result;

    result = CompareCameraInfo(
        sensor_->depth_camera_info(), depth_camera_.core().intrinsics());
    if (!result) return result;

    result = Compare(sensor_->depth_render_camera(), depth_camera_);
    if (!result) return result;

    // By default, frames B, C, and D are aligned and coincident.
    EXPECT_TRUE(CompareMatrices(sensor_->X_BC().GetAsMatrix4(),
                                RigidTransformd().GetAsMatrix4()));
    EXPECT_TRUE(CompareMatrices(sensor_->X_BD().GetAsMatrix4(),
                                RigidTransformd().GetAsMatrix4()));

    // Confirm the pose used by the renderer is the expected X_WC pose. We do
    // this by invoking a render (the dummy render engine will cache the last
    // call to UpdateViewpoint()).
    if (pre_render_callback) pre_render_callback();
    sensor_->color_image_output_port().Eval<ImageRgba8U>(*sensor_context_);
    EXPECT_TRUE(CompareMatrices(
        render_engine_->last_updated_X_WC().GetAsMatrix4(),
        X_WC_expected.GetAsMatrix4()));

    return result;
  }

  ColorRenderCamera color_camera_;
  DepthRenderCamera depth_camera_;
  unique_ptr<Diagram<double>> diagram_;
  unique_ptr<Context<double>> context_;

  // Convenient pointers into the diagram and context; the underlying systems
  // are owned by the diagram and its context.
  SceneGraph<double>* scene_graph_{};
  RgbdSensor* sensor_{};
  const DummyRenderEngine* render_engine_{};
  Context<double>* sensor_context_{};
  Context<double>* scene_graph_context_{};

  static const char kRendererName[];
};

const char RgbdSensorTest::kRendererName[] = "renderer";

// Confirms that port names are as documented in rgbd_sensor.h. This uses the
// anchored constructor and assumes that the ports are the same for the
// frame-fixed port.
TEST_F(RgbdSensorTest, PortNames) {
  RgbdSensor sensor(SceneGraph<double>::world_frame_id(),
                    RigidTransformd::Identity(), depth_camera_);
  EXPECT_EQ(sensor.query_object_input_port().get_name(), "geometry_query");
  EXPECT_EQ(sensor.color_image_output_port().get_name(), "color_image");
  EXPECT_EQ(sensor.depth_image_32F_output_port().get_name(), "depth_image_32f");
  EXPECT_EQ(sensor.depth_image_16U_output_port().get_name(), "depth_image_16u");
  EXPECT_EQ(sensor.label_image_output_port().get_name(), "label_image");
  EXPECT_EQ(sensor.body_pose_in_world_output_port().get_name(),
            "body_pose_in_world");
}

// Tests that the anchored camera reports the correct parent frame and has the
// right pose passed to the renderer.
TEST_F(RgbdSensorTest, ConstructAnchoredCamera) {
  const Vector3d p_WB(1, 2, 3);
  const RollPitchYawd rpy_WB(M_PI / 2, 0, 0);
  const RigidTransformd X_WB(rpy_WB, p_WB);

  auto make_sensor = [this, &X_WB](SceneGraph<double>*) {
    return make_unique<RgbdSensor>(SceneGraph<double>::world_frame_id(), X_WB,
                                   color_camera_, depth_camera_);
  };
  MakeCameraDiagram(make_sensor);

  const RigidTransformd& X_BC = sensor_->X_BC();
  const RigidTransformd X_WC_expected = X_WB * X_BC;
  EXPECT_TRUE(
      ValidateConstruction(scene_graph_->world_frame_id(), X_WC_expected));
}

// Similar to the AnchoredCamera test -- but, in this case, the reported pose
// of the camera X_WC depends on the value of the specified parent frame P.
TEST_F(RgbdSensorTest, ConstructFrameFixedCamera) {
  SourceId source_id;
  const GeometryFrame frame("camera_frame");
  const RigidTransformd X_PB(AngleAxisd(M_PI/6, Vector3d(1, 1, 1)),
                             Vector3d(1, 2, 3));
  const RigidTransformd X_WP(AngleAxisd(M_PI/7, Vector3d(-1, 0, 1)),
                             Vector3d(-2, -1, -3));
  const FramePoseVector<double> X_WPs{{frame.id(), X_WP}};

  // The sensor requires a frame to be registered in order to attach to the
  // frame.
  auto make_sensor = [this, &source_id, &frame,
                      &X_PB](SceneGraph<double>* graph) {
    source_id = graph->RegisterSource("source");
    graph->RegisterFrame(source_id, frame);
    return make_unique<RgbdSensor>(frame.id(), X_PB, color_camera_,
                                   depth_camera_);
  };
  MakeCameraDiagram(make_sensor);

  const RigidTransformd& X_BC = sensor_->X_BC();
  // NOTE: This *particular* factorization eliminates the need for a tolerance
  // in the matrix comparison -- it is the factorization that is implicit in
  // the code path for rendering.
  const RigidTransformd X_WC_expected = X_WP * (X_PB * X_BC);
  auto pre_render_callback = [this, &X_WPs, source_id]() {
    scene_graph_->get_source_pose_port(source_id).FixValue(scene_graph_context_,
                                                           X_WPs);
  };
  EXPECT_TRUE(
      ValidateConstruction(frame.id(), X_WC_expected, pre_render_callback));
}

TEST_F(RgbdSensorTest, ConstructCameraWithNonTrivialOffsets) {
  const RigidTransformd X_BC{
        math::RotationMatrixd::MakeFromOrthonormalRows(
            Eigen::Vector3d(0, 0, 1),
            Eigen::Vector3d(-1, 0, 0),
            Eigen::Vector3d(0, -1, 0)),
        Eigen::Vector3d(0, 0.02, 0)};
  // For uniqueness, simply invert X_BC.
  const RigidTransformd X_BD{X_BC.inverse()};
  const ColorRenderCamera color_camera{
      {color_camera_.core().renderer_name(), color_camera_.core().intrinsics(),
       color_camera_.core().clipping(), X_BC},
      color_camera_.show_window()};
  const DepthRenderCamera depth_camera{
    {depth_camera_.core().renderer_name(), depth_camera_.core().intrinsics(),
       depth_camera_.core().clipping(), X_BD},
      depth_camera_.depth_range()};
  const RigidTransformd X_WB;
  const RgbdSensor sensor(scene_graph_->world_frame_id(), X_WB, color_camera,
                          depth_camera);
  EXPECT_TRUE(CompareMatrices(
      sensor.X_BC().GetAsMatrix4(), X_BC.GetAsMatrix4()));
  EXPECT_TRUE(CompareMatrices(
      sensor.X_BD().GetAsMatrix4(), X_BD.GetAsMatrix4()));
}

TEST_F(RgbdSensorTest, ConstructCameraWithNonTrivialOffsetsDeprecated) {
  const RigidTransformd X_BC{
        math::RotationMatrixd::MakeFromOrthonormalRows(
            Eigen::Vector3d(0, 0, 1),
            Eigen::Vector3d(-1, 0, 0),
            Eigen::Vector3d(0, -1, 0)),
        Eigen::Vector3d(0, 0.02, 0)};
  // For uniqueness, simply invert X_BC.
  const RigidTransformd X_BD{X_BC.inverse()};
  const RigidTransformd X_WB;
  const ColorRenderCamera color_camera(
      {color_camera_.core().renderer_name(),
       {color_camera_.core().intrinsics().width(),
        color_camera_.core().intrinsics().height(),
        color_camera_.core().intrinsics().fov_y()},
       color_camera_.core().clipping(),
       X_BC},
      false);
  const DepthRenderCamera depth_camera(
      {depth_camera_.core().renderer_name(),
       {depth_camera_.core().intrinsics().width(),
        depth_camera_.core().intrinsics().height(),
        depth_camera_.core().intrinsics().fov_y()},
       depth_camera_.core().clipping(),
       X_BD},
      depth_camera_.depth_range());
  const RgbdSensor sensor(scene_graph_->world_frame_id(), X_WB, color_camera,
                          depth_camera);
  EXPECT_TRUE(CompareMatrices(
      sensor.X_BC().GetAsMatrix4(), X_BC.GetAsMatrix4()));
  EXPECT_TRUE(CompareMatrices(
      sensor.X_BD().GetAsMatrix4(), X_BD.GetAsMatrix4()));
}

// We don't explicitly test any of the image outputs. Most of the image outputs
// simple wrap the corresponding QueryObject call; the only calculations they
// do is to produce the X_PC matrix (which is implicitly tested in the
// construction tests above). The *one* exception is the 16U depth image which
// converts 32F to 16U. In this case, we test the conversion method directly.
TEST_F(RgbdSensorTest, DepthImage32FTo16U) {
  // The largest uint16_t value (unitless). Period.
  const uint16_t kMax16 = std::numeric_limits<uint16_t>::max();
  // The saturation depth (in mm) that will be reported.
  const double kDepth16Saturation = kMax16 / 1000.0;

  // Create a simple depth image with some representative values and their
  // corresponding uint16 values.
  std::array<uint16_t, 4> expected_depths;
  ImageDepth32F depth32(1, 4);
  // Too near.
  *depth32.at(0, 0) = 0.0;
  expected_depths[0] = 0;
  // Valid and representable.
  *depth32.at(0, 1) = kDepth16Saturation / 2;
  expected_depths[1] = kMax16 / 2;
  // Valid but not representable.
  *depth32.at(0, 2) = kDepth16Saturation * 1.1;
  expected_depths[2] = kMax16;
  // Too far.
  *depth32.at(0, 3) = std::numeric_limits<float>::infinity();
  expected_depths[3] = kMax16;

  // Perform conversion.
  ImageDepth16U depth16(1, 4);
  RgbdSensorTester::ConvertDepth32FTo16U(depth32, &depth16);

  for (int c = 0; c < 4; ++c) {
    EXPECT_EQ(*depth16.at(0, c), expected_depths[c]);
  }
}

// Tests that the discrete sensor is properly constructed.
GTEST_TEST(RgbdSensorDiscrete, Construction) {
  const DepthRenderCamera depth_camera(
      {"render", {640, 480, M_PI / 4}, {0.1, 10.0}, {}}, {0.1, 10});
  const double kPeriod = 0.1;

  const bool include_render_port = true;
  // N.B. In addition to testing a discrete sensor, this also tests
  // the `RgbdSensor` constructor which takes only `DepthRenderCamera`.
  RgbdSensorDiscrete sensor(
      make_unique<RgbdSensor>(SceneGraph<double>::world_frame_id(),
                              RigidTransformd::Identity(), depth_camera),
      kPeriod, include_render_port);
  EXPECT_EQ(sensor.query_object_input_port().get_name(), "geometry_query");
  EXPECT_EQ(sensor.color_image_output_port().get_name(), "color_image");
  EXPECT_EQ(sensor.depth_image_32F_output_port().get_name(), "depth_image_32f");
  EXPECT_EQ(sensor.depth_image_16U_output_port().get_name(), "depth_image_16u");
  EXPECT_EQ(sensor.label_image_output_port().get_name(), "label_image");
  EXPECT_EQ(sensor.body_pose_in_world_output_port().get_name(),
            "body_pose_in_world");

  // Confirm that the period was passed into the ZOH correctly. If the ZOH
  // reports the expected period, we rely on it to do the right thing.
  EXPECT_EQ(sensor.period(), kPeriod);
}

// Test that the diagram's internal architecture is correct and, likewise,
// wired correctly.
GTEST_TEST(RgbdSensorDiscrete, ImageHold) {
  const DepthRenderCamera depth_camera(
      {"render", {640, 480, M_PI / 4}, {0.1, 10.0}, {}}, {0.1, 10});
  // N.B. In addition to testing a discrete sensor, this also tests
  // the `RgbdSensor` constructor which takes only `DepthRenderCamera`.
  auto sensor =
      make_unique<RgbdSensor>(SceneGraph<double>::world_frame_id(),
                              RigidTransformd::Identity(), depth_camera);
  RgbdSensor* sensor_raw = sensor.get();
  const double kPeriod = 0.1;
  const bool include_render_port = true;
  RgbdSensorDiscrete discrete_sensor(move(sensor), kPeriod,
                                     include_render_port);

  // This tests very *explicit* knowledge of what the wiring should be. As such,
  // it's a bit brittle, but this is the most efficient way to affect this test.
  // We assume these systems are reported in the order they were added.
  vector<const System<double>*> sub_systems = discrete_sensor.GetSystems();

  // Five sub-systems: the sensor and one hold per image type.
  ASSERT_EQ(sub_systems.size(), 5);
  ASSERT_EQ(sub_systems[0], sensor_raw);

  // For each image output port, we want to make sure it's connected to the
  // expected ZOH and that the hold's period is kPeriod. This proves that
  // RgbdSensorDiscrete has wired things up properly.
  auto confirm_hold = [&sub_systems, &kPeriod, &discrete_sensor](
                          int hold_index,
                          const OutputPort<double>& image_port) {
    const ZeroOrderHold<double>* zoh =
        dynamic_cast<const ZeroOrderHold<double>*>(sub_systems[hold_index]);
    ASSERT_NE(zoh, nullptr);
    EXPECT_EQ(zoh->period(), kPeriod);
    EXPECT_TRUE(
        discrete_sensor.AreConnected(image_port, zoh->get_input_port()));
  };

  confirm_hold(1, sensor_raw->color_image_output_port());
  confirm_hold(2, sensor_raw->depth_image_32F_output_port());
  confirm_hold(3, sensor_raw->depth_image_16U_output_port());
  confirm_hold(4, sensor_raw->label_image_output_port());

  // TODO(SeanCurtis-TRI): Consider confirming that the exported ports map to
  //  the expected sub-system ports.
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
