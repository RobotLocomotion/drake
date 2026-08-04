#include "drake/systems/sensors/sim_rgbd_sensor.h"

#include <string>
#include <utility>

#include <fmt/format.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/rgbd_sensor.h"

namespace drake {
namespace systems {
namespace sensors {
namespace internal {
namespace {

// We need drake:: because there's also a systems::lcm namespace.
using drake::lcm::DrakeLcm;
using Eigen::Vector3d;
using geometry::SceneGraph;
using geometry::render::ClippingRange;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRange;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderCameraCore;
using math::RigidTransformd;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::FixedOffsetFrame;
using multibody::Frame;
using multibody::MultibodyPlant;
using multibody::RigidBody;
using systems::lcm::LcmPublisherSystem;

/* Returns a pointer to the named instance of TargetSystem (if it exists). */
template <typename TargetSystem>
const TargetSystem* GetSystem(const DiagramBuilder<double>& builder,
                              const std::string& name) {
  for (const auto* system : builder.GetSystems()) {
    if (system->get_name() == name) {
      const TargetSystem* result = dynamic_cast<const TargetSystem*>(system);
      EXPECT_NE(result, nullptr);
      return result;
    }
  }
  return nullptr;
}

std::pair<ColorRenderCamera, DepthRenderCamera> MakeCameras() {
  const RenderCameraCore core("dummy_renderer", CameraInfo(320, 240, 0.75),
                              ClippingRange{10, 20}, {});
  return {ColorRenderCamera(core, true),
          DepthRenderCamera(core, DepthRange{11.5, 13.5})};
}

class SimRgbdSensorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder_, 1e-2);
    bodyA_ = &plant_->AddRigidBody("bodyA");
    bodyB_ = &plant_->AddRigidBody("bodyB");

    X_AF_ = RigidTransformd(Vector3d(-0.5, 0.5, 0.75));
    frame_F_ = &plant_->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
        "test_frame", *bodyA_, X_AF_));

    plant_->Finalize();
  }

  /* Constructs a default sensor specification. If no frame is given, the sensor
   is affixed to body A (bodyA_). */
  SimRgbdSensor MakeSensorSpec(const Frame<double>* frame = nullptr) {
    const RigidTransformd X_AB(Vector3d(1, 2, 3));
    const auto [color, depth] = MakeCameras();
    return SimRgbdSensor("test1", frame ? *frame : bodyA_->body_frame(), 32.0,
                         X_AB, color, depth);
  }

  /* Adds an RgbdSensor to builder_. Returns a pointer to the RgbdSensor and
   the SimRgbdSensor that created it. If no frame is given, the sensor is
   affixed to body A (bodyA_).
   @throws std::exception if the RgbdSensor wasn't added. */
  const std::pair<SimRgbdSensor, const RgbdSensor*> MakeSensorOrThrow(
      const Frame<double>* frame = nullptr) {
    SimRgbdSensor sensor = MakeSensorSpec(frame);

    AddSimRgbdSensor(*scene_graph_, *plant_, sensor, &builder_);

    // Get the RgbdSensor from the builder.
    const auto* rgbd =
        GetSystem<RgbdSensor>(builder_, "rgbd_sensor_" + sensor.serial());
    if (rgbd == nullptr) {
      throw std::logic_error("Failed to instantiate RgbdSensor");
    }
    return {sensor, rgbd};
  }

  /* Test helper for determining whether specifying an image port produces the
   expected connections. */
  void AssertPublishedPorts(bool rgb, bool depth, bool label) {
    auto [sensor, rgbd] = MakeSensorOrThrow();
    const size_t old_system_count = builder_.GetSystems().size();
    AddSimRgbdSensorLcmPublisher(
        sensor, rgb ? &rgbd->color_image_output_port() : nullptr,
        depth ? &rgbd->depth_image_16U_output_port() : nullptr,
        label ? &rgbd->label_image_output_port() : nullptr, false, &builder_,
        &lcm_);
    EXPECT_LT(old_system_count, builder_.GetSystems().size());

    const auto* images = GetSystem<ImageToLcmImageArrayT>(
        builder_, "image_to_lcm_" + sensor.serial());
    ASSERT_NE(images, nullptr);
    if (rgb) {
      EXPECT_NO_THROW(images->GetInputPort("rgb"));
    } else {
      EXPECT_THROW(images->GetInputPort("rgb"), std::exception);
    }
    if (depth) {
      EXPECT_NO_THROW(images->GetInputPort("depth"));
    } else {
      EXPECT_THROW(images->GetInputPort("depth"), std::exception);
    }
    if (label) {
      EXPECT_NO_THROW(images->GetInputPort("label"));
    } else {
      EXPECT_THROW(images->GetInputPort("label"), std::exception);
    }
  }

  DiagramBuilder<double> builder_;
  MultibodyPlant<double>* plant_{};
  SceneGraph<double>* scene_graph_{};
  const RigidBody<double>* bodyA_{};
  const RigidBody<double>* bodyB_{};
  const Frame<double>* frame_F_{};
  RigidTransformd X_AF_;
  DrakeLcm lcm_;
};

/* Simply tests that all values passed in are available. We'll create two
 different instances with arbitrarily different values and confirm that the
 values are returned. */
TEST_F(SimRgbdSensorTest, SimRgbdSensorValues) {
  // The values for the camera core are irrelevant to the test -- to confirm
  // that camera properties get copied, we'll just look for unique values
  // at the highest level (i.e., color and depth) and assume that if *those*
  // values get copied, then all of the values got copied.
  const RenderCameraCore core("dummy_renderer", CameraInfo(320, 240, 0.75),
                              ClippingRange{10, 20}, {});
  {
    const ColorRenderCamera color(core, true);
    const DepthRenderCamera depth(core, DepthRange{11.5, 13.5});
    const RigidTransformd X_PB(Vector3d(1, 2, 3));
    const Frame<double>& frame = bodyA_->body_frame();
    SimRgbdSensor sensor("test1", frame, 32.0, X_PB, color, depth);
    EXPECT_EQ("test1", sensor.serial());
    EXPECT_EQ(&frame, &sensor.frame());
    EXPECT_TRUE(drake::CompareMatrices(X_PB.GetAsMatrix34(),
                                       sensor.X_PB().GetAsMatrix34()));
    EXPECT_TRUE(sensor.color_properties().show_window());
    EXPECT_EQ(11.5, sensor.depth_properties().depth_range().min_depth());
    EXPECT_EQ(32.0, sensor.rate_hz());
  }

  {
    const ColorRenderCamera color(core, false);
    const DepthRenderCamera depth(core, DepthRange{13.5, 15.5});
    const RigidTransformd X_PB(Vector3d(10, 20, 30));
    const Frame<double>& frame = bodyB_->body_frame();
    SimRgbdSensor sensor("test2", frame, 64.0, X_PB, color, depth);
    EXPECT_EQ("test2", sensor.serial());
    EXPECT_EQ(&frame, &sensor.frame());
    EXPECT_TRUE(drake::CompareMatrices(X_PB.GetAsMatrix34(),
                                       sensor.X_PB().GetAsMatrix34()));
    EXPECT_FALSE(sensor.color_properties().show_window());
    EXPECT_EQ(13.5, sensor.depth_properties().depth_range().min_depth());
    EXPECT_EQ(64.0, sensor.rate_hz());
  }
}

/* Confirms that when the frame is a body frame, that the RgbdSensor is affixed
 to the expected *geometry* frame with the expected pose. */
TEST_F(SimRgbdSensorTest, AddSensorWithBodyFrameSpecification) {
  auto [sensor, rgbd] = MakeSensorOrThrow();
  EXPECT_EQ(rgbd->default_parent_frame_id(),
            plant_->GetBodyFrameIdOrThrow(bodyA_->index()));

  auto diagram = builder_.Build();
  auto context = diagram->CreateDefaultContext();
  auto& plant_context = plant_->GetMyMutableContextFromRoot(context.get());
  const RigidTransformd X_WA(Vector3d(-3, -4, -5));
  plant_->SetFreeBodyPose(&plant_context, *bodyA_, X_WA);

  const auto& rgbd_context = rgbd->GetMyContextFromRoot(*context);
  const auto& X_WB =
      rgbd->body_pose_in_world_output_port().Eval<RigidTransformd>(
          rgbd_context);

  const RigidTransformd& X_AB = sensor.X_PB();
  EXPECT_TRUE(drake::CompareMatrices(X_WB.GetAsMatrix34(),
                                     (X_WA * X_AB).GetAsMatrix34()));
}

/* Confirms that when the frame is a secondary frame P, that the RgbdSensor is
 affixed to the *geometry* frame associated with the secondary frame's body A
 and with a pose that accounts for both X_PB and X_AP. */
TEST_F(SimRgbdSensorTest, AddSensorWithBodyOffsetFrameSpecification) {
  auto [sensor, rgbd] = MakeSensorOrThrow(frame_F_);
  EXPECT_EQ(rgbd->default_parent_frame_id(),
            plant_->GetBodyFrameIdOrThrow(bodyA_->index()));

  auto diagram = builder_.Build();
  auto context = diagram->CreateDefaultContext();
  auto& plant_context = plant_->GetMyMutableContextFromRoot(context.get());
  const RigidTransformd X_WA(Vector3d(-3, -4, -5));
  plant_->SetFreeBodyPose(&plant_context, *bodyA_, X_WA);

  const auto& rgbd_context = rgbd->GetMyContextFromRoot(*context);
  const auto& X_WB =
      rgbd->body_pose_in_world_output_port().Eval<RigidTransformd>(
          rgbd_context);

  const RigidTransformd& X_FB = sensor.X_PB();
  EXPECT_TRUE(drake::CompareMatrices(X_WB.GetAsMatrix34(),
                                     (X_WA * X_AF_ * X_FB).GetAsMatrix34()));
}

/* Confirms that the added RgbdSensor has been properly configured:

     - it has been named based on serial,
     - its query object port is connected, and
     - the camera properties match the input specification. */
TEST_F(SimRgbdSensorTest, AddSensorForRgbdSensorConfiguration) {
  auto [sensor, rgbd] = MakeSensorOrThrow();

  // Test criteria.
  EXPECT_THAT(rgbd->get_name(), ::testing::HasSubstr(sensor.serial()));
  EXPECT_TRUE(builder_.IsConnectedOrExported(rgbd->query_object_input_port()));
  // Relying on glass-box testing, the cameras should simply use copy semantics.
  // So, we'll test one field (renderer_name) and infer that the whole thing
  // got copied.
  EXPECT_EQ(rgbd->default_color_render_camera().core().renderer_name(),
            sensor.color_properties().core().renderer_name());
  EXPECT_EQ(rgbd->default_depth_render_camera().core().renderer_name(),
            sensor.depth_properties().core().renderer_name());
}

/* Confirms that if no ports have been provided in the invocation, that the
 builder remains unchanged. In fact, we don't even have to have an RgbdSensor
 instantiated. */
TEST_F(SimRgbdSensorTest, AddPublisherNoPortIsNoOp) {
  const size_t old_system_count = builder_.GetSystems().size();
  SimRgbdSensor sensor = MakeSensorSpec();
  AddSimRgbdSensorLcmPublisher(sensor, nullptr, nullptr, nullptr, false,
                               &builder_, &lcm_);
  EXPECT_EQ(old_system_count, builder_.GetSystems().size());
}

/* Confirms that if only the rgb port is provided, only the rgb port is
 connected. */
TEST_F(SimRgbdSensorTest, AddPublisherRgbOnly) {
  AssertPublishedPorts(true /* rgb */, false /* depth */, false /* label */);
}

/* Confirms that if only the depth port is provided, only the depth port is
 connected. */
TEST_F(SimRgbdSensorTest, AddPublisherDepthOnly) {
  AssertPublishedPorts(false /* rgb */, true /* depth */, false /* label */);
}

/* Confirms that if only the label port is provided, only the label port is
 connected. */
TEST_F(SimRgbdSensorTest, AddPublisherLabelOnly) {
  AssertPublishedPorts(false /* rgb */, false /* depth */, true /* label */);
}

/* Confirms that if all the ports are provided, all should be connected. */
TEST_F(SimRgbdSensorTest, AddPublisherRgbDepthAndLabel) {
  AssertPublishedPorts(true /* rgb */, true /* depth */, true /* label */);
}

/* Confirms that publisher is configured properly (channel name and period). We
 don't test that `do_compress` is passed because the ImageToLcmImageArrayT class
 provides no introspection. */
TEST_F(SimRgbdSensorTest, AddPublisherTestProperties) {
  const auto [sensor, rgbd] = MakeSensorOrThrow();
  const size_t old_system_count = builder_.GetSystems().size();
  AddSimRgbdSensorLcmPublisher(sensor, &rgbd->color_image_output_port(),
                               &rgbd->depth_image_16U_output_port(),
                               &rgbd->label_image_output_port(), false,
                               &builder_, &lcm_);
  EXPECT_LT(old_system_count, builder_.GetSystems().size());

  const auto* publisher = GetSystem<LcmPublisherSystem>(
      builder_, fmt::format("LcmPublisherSystem(DRAKE_RGBD_CAMERA_IMAGES_{})",
                            sensor.serial()));
  ASSERT_NE(publisher, nullptr);
  EXPECT_DOUBLE_EQ(publisher->get_publish_period(), 1.0 / sensor.rate_hz());
  EXPECT_THAT(publisher->get_channel_name(),
              ::testing::HasSubstr(sensor.serial()));
}

}  // namespace
}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
