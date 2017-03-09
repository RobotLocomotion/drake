#include "drake/automotive/maliput_railcar_to_euler_floating_joint.h"

#include <memory>

#include "gtest/gtest.h"

#include "drake/automotive/gen/maliput_railcar_config.h"
#include "drake/automotive/maliput/dragway/lane.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/common/drake_path.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/parameters.h"

namespace drake {

using maliput::monolane::ArcOffset;
using maliput::monolane::Endpoint;
using maliput::monolane::EndpointXy;
using maliput::monolane::EndpointZ;

namespace automotive {
namespace {

GTEST_TEST(MaliputRailcarToEulerFloatingJointTest, TestUsingDragway) {
  auto dragway = std::make_unique<const maliput::dragway::RoadGeometry>(
      maliput::api::RoadGeometryId({"RailcarTestDragway"}), 1 /* num lanes */,
      100 /* length */, 3 /* lane width */, 2 /* shoulder width */);
  const MaliputRailcarConfig<double> config;

  auto dut = std::make_unique<MaliputRailcarToEulerFloatingJoint<double>>(
      *dragway->junction(0)->segment(0)->lane(0), config);
  auto context = dut->CreateDefaultContext();
  auto output = dut->AllocateOutput(*context);

  auto value = std::make_unique<MaliputRailcarState<double>>();
  value->set_s(2);
  value->set_speed(1);
  context->SetInputPort(
      dut->input_port_index(),
      std::make_unique<systems::FreestandingInputPort>(std::move(value)));

  // Grabs a pointer to where the CalcOutput results end up.
  const EulerFloatingJointState<double>* const result =
      dynamic_cast<const EulerFloatingJointState<double>*>(
          output->get_vector_data(dut->output_port_index()));
  ASSERT_NE(result, nullptr);

  dut->CalcOutput(*context, output.get());
  EXPECT_EQ(result->x(), 2);
  EXPECT_EQ(result->y(), 0);
  EXPECT_EQ(result->z(), 0);
  EXPECT_EQ(result->yaw(), 0);
  EXPECT_EQ(result->pitch(), 0);
  EXPECT_EQ(result->roll(), 0);
}

std::unique_ptr<const maliput::api::RoadGeometry> CreateCurvedRoad(
    double radius, double theta) {
  maliput::monolane::Builder builder(
      maliput::api::RBounds(-2, 2),   /* lane_bounds       */
      maliput::api::RBounds(-4, 4),   /* driveable_bounds  */
      0.01,                           /* linear tolerance  */
      0.5 * M_PI / 180.0);            /* angular_tolerance */
  builder.Connect(
      "point.0",                                             /* id    */
      Endpoint(EndpointXy(0, 0, 0), EndpointZ(0, 0, 0, 0)),  /* start */
      ArcOffset(radius, theta),                              /* arc   */
      EndpointZ(0, 0, 0, 0));                                /* z_end */
  return builder.Build(maliput::api::RoadGeometryId({"RailcarTestCurvedRoad"}));
}

void EvaluateDutUsingMonolane(double r) {
  const double kCurvedRoadRadius{10};
  const double kCurvedRoadTheta{M_PI_2};
  const auto curved_road =
      CreateCurvedRoad(kCurvedRoadRadius, kCurvedRoadTheta);

  MaliputRailcarConfig<double> config;
  config.set_r(r);
  auto dut = std::make_unique<MaliputRailcarToEulerFloatingJoint<double>>(
      *curved_road->junction(0)->segment(0)->lane(0),
      config);
  auto context = dut->CreateDefaultContext();
  auto output = dut->AllocateOutput(*context);

  auto value = std::make_unique<MaliputRailcarState<double>>();
  value->set_s(2 * M_PI * kCurvedRoadRadius / 4);
  value->set_speed(1);
  context->SetInputPort(
      dut->input_port_index(),
      std::make_unique<systems::FreestandingInputPort>(std::move(value)));

  // Grabs a pointer to where the CalcOutput results end up.
  const EulerFloatingJointState<double>* const result =
      dynamic_cast<const EulerFloatingJointState<double>*>(
          output->get_vector_data(dut->output_port_index()));
  ASSERT_NE(result, nullptr);

  dut->CalcOutput(*context, output.get());
  EXPECT_EQ(result->x(), kCurvedRoadRadius - r);
  EXPECT_EQ(result->y(), kCurvedRoadRadius);
  EXPECT_EQ(result->z(), 0);
  EXPECT_DOUBLE_EQ(result->yaw(), M_PI_2);
  EXPECT_EQ(result->pitch(), 0);
  EXPECT_EQ(result->roll(), 0);
}

GTEST_TEST(MaliputRailcarToEulerFloatingJointTest, TestUsingMonolaneZeroR) {
  EvaluateDutUsingMonolane(0);
}

GTEST_TEST(MaliputRailcarToEulerFloatingJointTest, TestUsingMonolaneNonZeroR) {
  EvaluateDutUsingMonolane(1);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
