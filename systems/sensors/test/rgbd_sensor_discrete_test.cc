#include "drake/systems/sensors/rgbd_sensor_discrete.h"

#include <gtest/gtest.h>

#include "drake/geometry/scene_graph.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {

using geometry::SceneGraph;
using geometry::render::DepthRenderCamera;
using math::RigidTransformd;
using std::make_unique;
using std::vector;

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
  RgbdSensorDiscrete discrete_sensor(std::move(sensor), kPeriod,
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
