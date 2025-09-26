#include "drake/systems/sensors/rgbd_sensor.h"

#include <functional>
#include <memory>
#include <string>
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

namespace drake {
namespace systems {
namespace sensors {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
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
using std::unique_ptr;
using std::vector;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;

std::ostream& operator<<(std::ostream& out, const CameraInfo& info) {
  out << "\n  width: " << info.width() << "\n  height: " << info.height()
      << "\n  focal_x: " << info.focal_x() << "\n  focal_y: " << info.focal_y()
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
  out << "DepthRenderCamera\n"
      << camera.core().intrinsics()
      << "\n  min_depth: " << camera.depth_range().min_depth()
      << "\n  max_depth: " << camera.depth_range().max_depth();
  return out;
}

namespace {

template <typename T>
const DummyRenderEngine* GetDummyRenderEngine(
    const systems::Context<T>& context, const std::string& name) {
  // Technically brittle, but relatively safe assumption that GeometryState
  // is abstract Parameter value 0.
  auto& geo_state =
      context.get_parameters()
          .template get_abstract_parameter<geometry::GeometryState<T>>(0);
  const DummyRenderEngine* engine = dynamic_cast<const DummyRenderEngine*>(
      geo_state.GetRenderEngineByName(name));
  DRAKE_DEMAND(engine != nullptr);
  return engine;
}

void CompareCameraInfo(const CameraInfo& test, const CameraInfo& expected) {
  EXPECT_EQ(test.width(), expected.width());
  EXPECT_EQ(test.height(), expected.height());
  EXPECT_EQ(test.focal_x(), expected.focal_x());
  EXPECT_EQ(test.focal_y(), expected.focal_y());
  EXPECT_EQ(test.center_x(), expected.center_x());
  EXPECT_EQ(test.center_y(), expected.center_y());
}

void CompareClipping(const ClippingRange& test, const ClippingRange& expected) {
  EXPECT_EQ(test.near(), expected.near());
  EXPECT_EQ(test.far(), expected.far());
}

void CompareDepthRange(const DepthRange& test, const DepthRange& expected) {
  EXPECT_EQ(test.min_depth(), expected.min_depth());
  EXPECT_EQ(test.max_depth(), expected.max_depth());
}

void CompareCameraCore(const RenderCameraCore& test,
                       const RenderCameraCore& expected) {
  SCOPED_TRACE("CompareCameraCore()");
  CompareCameraInfo(test.intrinsics(), expected.intrinsics());

  EXPECT_EQ(test.renderer_name(), expected.renderer_name());
  CompareClipping(test.clipping(), expected.clipping());

  EXPECT_TRUE(
      CompareMatrices(test.sensor_pose_in_camera_body().GetAsMatrix4(),
                      expected.sensor_pose_in_camera_body().GetAsMatrix4()));
}

void Compare(const ColorRenderCamera& test, const ColorRenderCamera& expected) {
  SCOPED_TRACE("Compare(ColorRenderCamera)");
  EXPECT_EQ(test.show_window(), expected.show_window());
  CompareCameraCore(test.core(), expected.core());
}

void Compare(const DepthRenderCamera& test, const DepthRenderCamera& expected) {
  SCOPED_TRACE("Compare(DepthRenderCamera)");
  CompareCameraCore(test.core(), expected.core());
  CompareDepthRange(test.depth_range(), expected.depth_range());
}

class RgbdSensorTest : public ::testing::Test {
 public:
  RgbdSensorTest()
      : ::testing::Test(),
        // N.B. This is using arbitrary yet different intrinsics for color vs.
        // depth. The pose of the sensor in the camera body frame is *not* the
        // identity. We want to make sure that the RgbdSensor doesn't make use
        // of it.
        color_camera_({kRendererName,
                       {640, 480, M_PI / 4},
                       {0.1, 10.0},
                       RigidTransformd{Vector3d{1, 0, 0}}},
                      false),
        depth_camera_({kRendererName,
                       {320, 240, M_PI / 6},
                       {0.1, 10.0},
                       RigidTransformd{Vector3d{-1, 0, 0}}},
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
    context_->SetTime(22.2);  // An arbitrary non-zero value.
    context_->DisableCaching();
    scene_graph_context_ =
        &diagram_->GetMutableSubsystemContext(*scene_graph_, context_.get());
    sensor_context_ =
        &diagram_->GetMutableSubsystemContext(*sensor_, context_.get());
    // Must get the render engine instance from the context itself.
    render_engine_ = GetDummyRenderEngine(*scene_graph_context_, kRendererName);
  }

  // Confirms that the member sensor_ matches the expected properties. Part
  // of this confirmation entails rendering the camera which *may* pull on
  // an input port. The optional `pre_render_callback` should do any work
  // necessary to make the input port viable.
  void ValidateConstruction(
      FrameId parent_id, const RigidTransformd& X_WC_expected,
      std::function<void()> pre_render_callback = {}) const {
    EXPECT_EQ(sensor_->default_parent_frame_id(), parent_id);
    EXPECT_EQ(sensor_->GetParentFrameId(*sensor_context_), parent_id);
    {
      SCOPED_TRACE("validate color camera construction");
      CompareCameraInfo(
          sensor_->default_color_render_camera().core().intrinsics(),
          color_camera_.core().intrinsics());

      {
        SCOPED_TRACE("to default");
        Compare(sensor_->default_color_render_camera(), color_camera_);
      }
      {
        SCOPED_TRACE("to parameter");
        Compare(sensor_->GetColorRenderCamera(*sensor_context_), color_camera_);
      }
    }

    {
      SCOPED_TRACE("validate depth camera construction");
      CompareCameraInfo(
          sensor_->default_depth_render_camera().core().intrinsics(),
          depth_camera_.core().intrinsics());

      {
        SCOPED_TRACE("to default");
        Compare(sensor_->default_depth_render_camera(), depth_camera_);
      }
      {
        SCOPED_TRACE("to parameter");
        Compare(sensor_->GetDepthRenderCamera(*sensor_context_), depth_camera_);
      }
    }

    EXPECT_TRUE(
        CompareMatrices(sensor_->default_X_PB().GetAsMatrix4(),
                        sensor_->GetX_PB(*sensor_context_).GetAsMatrix4()));

    // Confirm the pose used by the renderer is the expected X_WC pose. We do
    // this by invoking a render (the dummy render engine will cache the last
    // call to UpdateViewpoint()).
    if (pre_render_callback) pre_render_callback();
    sensor_->color_image_output_port().Eval<ImageRgba8U>(*sensor_context_);
    EXPECT_TRUE(
        CompareMatrices(render_engine_->last_updated_X_WC().GetAsMatrix4(),
                        X_WC_expected.GetAsMatrix4(), 1e-15));
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
  EXPECT_EQ(sensor.image_time_output_port().get_name(), "image_time");
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

  const RigidTransformd& X_BC = sensor_->default_color_render_camera()
                                    .core()
                                    .sensor_pose_in_camera_body();
  const RigidTransformd X_WC_expected = X_WB * X_BC;
  ValidateConstruction(scene_graph_->world_frame_id(), X_WC_expected);
  EXPECT_EQ(sensor_->image_time_output_port().Eval(*sensor_context_),
            Vector1d{22.2});
}

// Similar to the AnchoredCamera test -- but, in this case, the reported pose
// of the camera X_WC depends on the value of the specified parent frame P.
TEST_F(RgbdSensorTest, ConstructFrameFixedCamera) {
  SourceId source_id;
  const GeometryFrame frame("camera_frame");
  const RigidTransformd X_PB(AngleAxisd(M_PI / 6, Vector3d(1, 1, 1)),
                             Vector3d(1, 2, 3));
  const RigidTransformd X_WP(AngleAxisd(M_PI / 7, Vector3d(-1, 0, 1)),
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

  const RigidTransformd& X_BC = sensor_->default_color_render_camera()
                                    .core()
                                    .sensor_pose_in_camera_body();
  // NOTE: This *particular* factorization eliminates the need for a tolerance
  // in the matrix comparison -- it is the factorization that is implicit in
  // the code path for rendering.
  const RigidTransformd X_WC_expected = X_WP * (X_PB * X_BC);
  auto pre_render_callback = [this, &X_WPs, source_id]() {
    scene_graph_->get_source_pose_port(source_id).FixValue(scene_graph_context_,
                                                           X_WPs);
  };
  ValidateConstruction(frame.id(), X_WC_expected, pre_render_callback);
}

TEST_F(RgbdSensorTest, ConstructCameraWithNonTrivialOffsets) {
  const RigidTransformd X_BC{
      math::RotationMatrixd::MakeFromOrthonormalRows(Eigen::Vector3d(0, 0, 1),
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
  EXPECT_TRUE(CompareMatrices(sensor.default_color_render_camera()
                                  .core()
                                  .sensor_pose_in_camera_body()
                                  .GetAsMatrix4(),
                              X_BC.GetAsMatrix4()));
  EXPECT_TRUE(CompareMatrices(sensor.default_depth_render_camera()
                                  .core()
                                  .sensor_pose_in_camera_body()
                                  .GetAsMatrix4(),
                              X_BD.GetAsMatrix4()));
}

TEST_F(RgbdSensorTest, Parameters) {
  // Construct an arbitrary sensor diagram, then change the sensor's defaults
  // and context parameters independently.
  const Vector3d p_WB(1, 2, 3);
  const RollPitchYawd rpy_WB(M_PI / 2, 0, 0);
  const RigidTransformd X_WB(rpy_WB, p_WB);

  auto make_sensor = [this, &X_WB](SceneGraph<double>*) {
    return make_unique<RgbdSensor>(SceneGraph<double>::world_frame_id(), X_WB,
                                   color_camera_, depth_camera_);
  };
  MakeCameraDiagram(make_sensor);
  const RigidTransformd& X_BC = sensor_->default_color_render_camera()
                                    .core()
                                    .sensor_pose_in_camera_body();
  const RigidTransformd X_WC_expected = X_WB * X_BC;
  // ValidateConstruction checks that defaults and parameters are the same.
  ValidateConstruction(scene_graph_->world_frame_id(), X_WC_expected);

  // Prepare some replacement value items.
  SourceId source_id;
  const GeometryFrame frame("some_frame");
  source_id = scene_graph_->RegisterSource("source");
  scene_graph_->RegisterFrame(source_id, frame);
  // Replacement cameras have arbitrary tiny width and height.
  ColorRenderCamera tiny_color_camera({kRendererName,
                                       {64, 48, M_PI / 4},
                                       {0.1, 10.0},
                                       RigidTransformd{Vector3d{1, 0, 0}}},
                                      false);
  DepthRenderCamera tiny_depth_camera({kRendererName,
                                       {32, 24, M_PI / 6},
                                       {0.1, 10.0},
                                       RigidTransformd{Vector3d{-1, 0, 0}}},
                                      {0.1, 10});

  // Define some local sugar to extract camera intrinsics width.
  auto width = [](const auto& camera) {
    return camera.core().intrinsics().width();
  };

  // Change the default values.
  sensor_->set_default_color_render_camera(tiny_color_camera);
  sensor_->set_default_depth_render_camera(tiny_depth_camera);
  sensor_->set_default_X_PB(RigidTransformd::Identity());
  sensor_->set_default_parent_frame_id(frame.id());

  // Defaults now differ from the parameters in the context created above.
  EXPECT_NE(width(sensor_->default_color_render_camera()),
            width(sensor_->GetColorRenderCamera(*sensor_context_)));
  EXPECT_NE(width(sensor_->default_depth_render_camera()),
            width(sensor_->GetDepthRenderCamera(*sensor_context_)));
  EXPECT_NE(sensor_->default_parent_frame_id(),
            sensor_->GetParentFrameId(*sensor_context_));
  EXPECT_FALSE(
      CompareMatrices(sensor_->default_X_PB().GetAsMatrix4(),
                      sensor_->GetX_PB(*sensor_context_).GetAsMatrix4()));

  // Create a new context with the changed default values.
  auto other_context = sensor_->CreateDefaultContext();

  // The new context matches the changed defaults.
  EXPECT_EQ(width(sensor_->default_color_render_camera()),
            width(sensor_->GetColorRenderCamera(*other_context)));
  EXPECT_EQ(width(sensor_->default_depth_render_camera()),
            width(sensor_->GetDepthRenderCamera(*other_context)));
  EXPECT_EQ(sensor_->default_parent_frame_id(),
            sensor_->GetParentFrameId(*other_context));
  EXPECT_TRUE(CompareMatrices(sensor_->default_X_PB().GetAsMatrix4(),
                              sensor_->GetX_PB(*other_context).GetAsMatrix4()));

  // Push the parameter values from the original context into the new context,
  // to test parameter mutating methods.
  sensor_->SetColorRenderCamera(
      other_context.get(), sensor_->GetColorRenderCamera(*sensor_context_));
  sensor_->SetDepthRenderCamera(
      other_context.get(), sensor_->GetDepthRenderCamera(*sensor_context_));
  sensor_->SetParentFrameId(other_context.get(),
                            sensor_->GetParentFrameId(*sensor_context_));
  sensor_->SetX_PB(other_context.get(), sensor_->GetX_PB(*sensor_context_));

  // Now, the new context differs from the changed defaults.
  EXPECT_NE(width(sensor_->default_color_render_camera()),
            width(sensor_->GetColorRenderCamera(*other_context)));
  EXPECT_NE(width(sensor_->default_depth_render_camera()),
            width(sensor_->GetDepthRenderCamera(*other_context)));
  EXPECT_NE(sensor_->default_parent_frame_id(),
            sensor_->GetParentFrameId(*other_context));
  EXPECT_FALSE(
      CompareMatrices(sensor_->default_X_PB().GetAsMatrix4(),
                      sensor_->GetX_PB(*other_context).GetAsMatrix4()));
}

TEST_F(RgbdSensorTest, ChangeImageSize) {
  // Construct a sensor diagram with arbitrary values.
  const Vector3d p_WB(1, 2, 3);
  const RollPitchYawd rpy_WB(M_PI / 2, 0, 0);
  const RigidTransformd X_WB(rpy_WB, p_WB);
  auto make_sensor = [this, &X_WB](SceneGraph<double>*) {
    return make_unique<RgbdSensor>(SceneGraph<double>::world_frame_id(), X_WB,
                                   color_camera_, depth_camera_);
  };
  MakeCameraDiagram(make_sensor);

  // Change the image size parameter in the context.
  const int new_width = 404;
  const int new_height = 202;
  const ColorRenderCamera old_color_camera =
      sensor_->GetColorRenderCamera(*sensor_context_);
  const DepthRenderCamera old_depth_camera =
      sensor_->GetDepthRenderCamera(*sensor_context_);
  const ColorRenderCamera new_color_camera{
      RenderCameraCore(old_color_camera.core().renderer_name(),
                       CameraInfo{new_width, new_height,
                                  old_color_camera.core().intrinsics().fov_y()},
                       old_color_camera.core().clipping(),
                       old_color_camera.core().sensor_pose_in_camera_body())};
  const DepthRenderCamera new_depth_camera{
      RenderCameraCore(old_depth_camera.core().renderer_name(),
                       CameraInfo{new_width, new_height,
                                  old_depth_camera.core().intrinsics().fov_y()},
                       old_depth_camera.core().clipping(),
                       old_depth_camera.core().sensor_pose_in_camera_body()),
      old_depth_camera.depth_range()};
  sensor_->SetColorRenderCamera(sensor_context_, new_color_camera);
  sensor_->SetDepthRenderCamera(sensor_context_, new_depth_camera);

  // Confirm that rendering uses the new image size.
  const auto& color_image =
      sensor_->color_image_output_port().Eval<ImageRgba8U>(*sensor_context_);
  EXPECT_EQ(color_image.width(), new_width);
  EXPECT_EQ(color_image.height(), new_height);
  const auto& depth_image_32 =
      sensor_->depth_image_32F_output_port().Eval<ImageDepth32F>(
          *sensor_context_);
  EXPECT_EQ(depth_image_32.width(), new_width);
  EXPECT_EQ(depth_image_32.height(), new_height);
  const auto& depth_image_16 =
      sensor_->depth_image_16U_output_port().Eval<ImageDepth16U>(
          *sensor_context_);
  EXPECT_EQ(depth_image_16.width(), new_width);
  EXPECT_EQ(depth_image_16.height(), new_height);
  const auto& label_image =
      sensor_->label_image_output_port().Eval<ImageLabel16I>(*sensor_context_);
  EXPECT_EQ(label_image.width(), new_width);
  EXPECT_EQ(label_image.height(), new_height);
}

TEST_F(RgbdSensorTest, ConstructCameraWithNonTrivialOffsetsDeprecated) {
  const RigidTransformd X_BC{
      math::RotationMatrixd::MakeFromOrthonormalRows(Eigen::Vector3d(0, 0, 1),
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
  EXPECT_TRUE(CompareMatrices(sensor.default_color_render_camera()
                                  .core()
                                  .sensor_pose_in_camera_body()
                                  .GetAsMatrix4(),
                              X_BC.GetAsMatrix4()));
  EXPECT_TRUE(CompareMatrices(sensor.default_depth_render_camera()
                                  .core()
                                  .sensor_pose_in_camera_body()
                                  .GetAsMatrix4(),
                              X_BD.GetAsMatrix4()));
}

// We don't explicitly test any of the image outputs (other than their size).
// The image outputs simply wrap the corresponding QueryObject call; the only
// calculations they do is to produce the X_PC matrix (which is implicitly
// tested in the construction tests above).

// TODO(jwnimmer-tri) The body_pose_in_world_output_port should have unit test
// coverage of its output value, not just its name. It ends up being indirectly
// tested in sim_rgbd_sensor_test.cc but it would be better to identify bugs in
// the RgbdSensor directly instead of intermingled with the wrapper code.

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
