#include "drake/systems/sensors/rgbd_sensor_discrete.h"

#include <memory>
#include <utility>
#include <vector>

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
  EXPECT_EQ(sensor.image_time_output_port().get_name(), "image_time");

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
  const int num_outputs = sensor_raw->num_output_ports();
  const double kPeriod = 0.1;
  const bool include_render_port = true;
  RgbdSensorDiscrete discrete_sensor(std::move(sensor), kPeriod,
                                     include_render_port);
  EXPECT_EQ(discrete_sensor.num_output_ports(), num_outputs);

  // This tests very *explicit* knowledge of what the wiring should be. As such,
  // it's a bit brittle, but this is the most efficient way to affect this test.
  // We assume these systems are reported in the order they were added.
  vector<const System<double>*> sub_systems = discrete_sensor.GetSystems();

  // For our diagram's sub-systems, we expect to have the sensor and then one
  // zero-order hold per output port.
  ASSERT_EQ(sub_systems.size(), 1 + num_outputs);
  ASSERT_EQ(sub_systems[0], sensor_raw);

  // For each output port, we want to make sure it's connected to the expected
  // zero-order hold and that the hold's period is kPeriod. This proves that
  // RgbdSensorDiscrete has wired things up properly.
  auto confirm_hold = [&sensor_raw, &sub_systems, &kPeriod,
                       &discrete_sensor](int port_index) {
    const auto* zoh = dynamic_cast<const ZeroOrderHold<double>*>(
        sub_systems.at(1 + port_index));
    ASSERT_NE(zoh, nullptr);
    EXPECT_EQ(zoh->period(), kPeriod);
    EXPECT_TRUE(discrete_sensor.AreConnected(
        sensor_raw->get_output_port(port_index), zoh->get_input_port()));
  };
  for (int i = 0; i < num_outputs; ++i) {
    SCOPED_TRACE(fmt::format("i = {}", i));
    confirm_hold(i);
  }

  // TODO(SeanCurtis-TRI): Consider confirming that the exported ports map to
  //  the expected sub-system ports.
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
