#include "drake/automotive/automotive_simulator.h"

#include <gtest/gtest.h>

#include "drake/automotive/curve2.h"
#include "drake/automotive/prius_vis.h"
#include "drake/common/drake_path.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcmt_simple_car_state_t.hpp"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/systems/framework/diagram_context.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {

using systems::Context;

namespace automotive {
namespace {

// Simple touches on the getters.
GTEST_TEST(AutomotiveSimulatorTest, BasicTest) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  EXPECT_NE(nullptr, simulator->get_lcm());
  EXPECT_NE(nullptr, simulator->get_builder());
}

// Obtains the serialized version of the last message transmitted on LCM channel
// @p channel. Uses @p translator to decode the message into @p joint_value.
void GetLastPublishedJointValue(
    const std::string& channel,
    const systems::lcm::LcmAndVectorBaseTranslator& translator,
    lcm::DrakeMockLcm* mock_lcm, EulerFloatingJointState<double>* joint_value) {
  const std::vector<uint8_t>& message =
      mock_lcm->get_last_published_message(channel);
  translator.Deserialize(message.data(), message.size(), joint_value);
}

// Covers AddPriusSimpleCar (and thus AddPublisher), Start, StepBy,
// GetSystemByName.
GTEST_TEST(AutomotiveSimulatorTest, TestPriusSimpleCar) {
  // TODO(jwnimmer-tri) Do something better than "0_" here.
  const std::string kJointStateChannelName = "0_FLOATING_JOINT_STATE";
  const std::string kCommandChannelName = "DRIVING_COMMAND";

  const std::string driving_command_name =
      systems::lcm::LcmSubscriberSystem::make_name(kCommandChannelName);
  const std::string joint_state_name =
      systems::lcm::LcmPublisherSystem::make_name(kJointStateChannelName);

  // Set up a basic simulation with just a Prius SimpleCar.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<lcm::DrakeMockLcm>());
  const int id = simulator->AddPriusSimpleCar("Foo", kCommandChannelName);
  EXPECT_EQ(id, 0);

  const int num_vis_elements = PriusVis<double>(0, "").num_poses();

  // Grab the systems we want while testing GetBuilderSystemByName() in the
  // process.
  auto& command_sub = dynamic_cast<systems::lcm::LcmSubscriberSystem&>(
      simulator->GetBuilderSystemByName(driving_command_name));
  auto& state_pub = dynamic_cast<systems::lcm::LcmPublisherSystem&>(
      simulator->GetBuilderSystemByName(joint_state_name));

  // Finish all initialization, so that we can test the post-init state.
  simulator->Start();

  // Set full throttle.
  DrivingCommand<double> command;
  command.set_acceleration(11.0);  // Arbitrary large positive.
  lcm::DrakeMockLcm* mock_lcm =
      dynamic_cast<lcm::DrakeMockLcm*>(simulator->get_lcm());
  ASSERT_NE(nullptr, mock_lcm);
  std::vector<uint8_t> message_bytes;
  command_sub.get_translator().Serialize(0.0 /* time */, command,
                                         &message_bytes);
  mock_lcm->InduceSubscriberCallback(kCommandChannelName, &message_bytes[0],
                                     message_bytes.size());

  // Shortly after starting, we should have not have moved much. Take two
  // small steps so that we get a publish a small time after zero (publish
  // occurs at the beginning of a step unless specific publishing times are
  // set).
  simulator->StepBy(0.005);
  simulator->StepBy(0.005);
  EulerFloatingJointState<double> joint_value;
  GetLastPublishedJointValue(kJointStateChannelName, state_pub.get_translator(),
                             mock_lcm, &joint_value);
  // The following is hard-coded to match prius.sdf and prius_with_lidar.sdf.
  const double kp_MoVo{1.40948};
  EXPECT_GT(joint_value.x() - kp_MoVo, 0.0);
  EXPECT_LT(joint_value.x() - kp_MoVo, 0.001);

  // Move a lot.  Confirm that we're moving in +x.
  for (int i = 0; i < 100; ++i) {
    simulator->StepBy(0.01);
  }
  // TODO(jwnimmer-tri) Check the timestamp of the final publication.
  GetLastPublishedJointValue(kJointStateChannelName, state_pub.get_translator(),
                             mock_lcm, &joint_value);
  EXPECT_GT(joint_value.x(), 1.0);

  // Confirm that appropriate draw messages are coming out. Just a few of the
  // message's fields are checked.
  const std::string channel_name = "DRAKE_VIEWER_DRAW";
  lcmt_viewer_draw published_draw_message =
      mock_lcm->DecodeLastPublishedMessageAs<lcmt_viewer_draw>(channel_name);

  EXPECT_EQ(published_draw_message.num_links, num_vis_elements);
  EXPECT_EQ(published_draw_message.link_name.at(0), "chassis_floor");
  EXPECT_EQ(published_draw_message.link_name.at(1), "front_axle");

  // The subsystem pointers must not change.
  EXPECT_EQ(&simulator->GetDiagramSystemByName(driving_command_name),
            &command_sub);
  EXPECT_EQ(&simulator->GetDiagramSystemByName(joint_state_name), &state_pub);
}

// Tests the ability to initialize a SimpleCar to a non-zero initial state.
GTEST_TEST(AutomotiveSimulatorTest, TestPriusSimpleCarInitialState) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<lcm::DrakeMockLcm>());
  const double kX{10};
  const double kY{5.5};
  const double kHeading{M_PI_2};
  const double kVelocity{4.5};

  SimpleCarState<double> initial_state;
  initial_state.set_x(kX);
  initial_state.set_y(kY);
  initial_state.set_heading(kHeading);
  initial_state.set_velocity(kVelocity);

  simulator->AddPriusSimpleCar(
      "My Test Model",
      "Channel",
      initial_state);
  simulator->Start();
  simulator->StepBy(1e-3);

  lcm::DrakeMockLcm* mock_lcm =
      dynamic_cast<lcm::DrakeMockLcm*>(simulator->get_lcm());
  ASSERT_NE(mock_lcm, nullptr);
  const lcmt_simple_car_state_t state_message =
      mock_lcm->DecodeLastPublishedMessageAs<lcmt_simple_car_state_t>(
          "0_SIMPLE_CAR_STATE");

  EXPECT_EQ(state_message.x, kX);
  EXPECT_EQ(state_message.y, kY);
  EXPECT_EQ(state_message.heading, kHeading);
  EXPECT_EQ(state_message.velocity, kVelocity);
}

// Cover AddTrajectoryCar (and thus AddPublisher).
GTEST_TEST(AutomotiveSimulatorTest, TestPriusTrajectoryCar) {
  typedef Curve2<double> Curve2d;
  typedef Curve2d::Point2 Point2d;
  const std::vector<Point2d> waypoints{
      {0.0, 0.0},
      {100.0, 0.0},
  };
  const Curve2d curve{waypoints};

  // Set up a basic simulation with a couple Prius TrajectoryCars. The first
  // car starts a time zero while the second starts at time 10. They both follow
  // a straight 100 m long line.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<lcm::DrakeMockLcm>());
  const int id1 = simulator->AddPriusTrajectoryCar(curve, 1.0, 0.0);
  const int id2 = simulator->AddPriusTrajectoryCar(curve, 1.0, 10.0);
  EXPECT_EQ(id1, 0);
  EXPECT_EQ(id2, 1);

  // Finish all initialization, so that we can test the post-init state.
  simulator->Start();

  // Simulate for one second.
  for (int i = 0; i < 100; ++i) {
    simulator->StepBy(0.01);
  }

  const lcm::DrakeLcmInterface* lcm = simulator->get_lcm();
  ASSERT_NE(lcm, nullptr);

  const lcm::DrakeMockLcm* mock_lcm =
      dynamic_cast<const lcm::DrakeMockLcm*>(lcm);
  ASSERT_NE(mock_lcm, nullptr);

  const int expected_num_links = PriusVis<double>(0, "").num_poses() * 2;

  // Verifies that the correct lcmt_viewer_load_robot message was transmitted.
  const lcmt_viewer_load_robot load_message =
      mock_lcm->DecodeLastPublishedMessageAs<lcmt_viewer_load_robot>(
          "DRAKE_VIEWER_LOAD_ROBOT");
  EXPECT_EQ(load_message.num_links, expected_num_links);

  struct LinkInfo {
    LinkInfo(std::string name_in, int robot_num_in, int num_geom_in)
      : name(name_in), robot_num(robot_num_in), num_geom(num_geom_in) {}
    std::string name;
    int robot_num{};
    int num_geom{};
  };

  const std::vector<LinkInfo> expected_load {
    LinkInfo("chassis_floor", 0, 1),
    LinkInfo("front_axle", 0, 1),
    LinkInfo("left_tie_rod_arm", 0, 2),
    LinkInfo("left_hub", 0, 1),
    LinkInfo("tie_rod", 0, 1),
    LinkInfo("left_wheel", 0, 3),
    LinkInfo("right_tie_rod_arm", 0, 2),
    LinkInfo("right_hub", 0, 1),
    LinkInfo("right_wheel", 0, 3),
    LinkInfo("rear_axle", 0, 1),
    LinkInfo("left_wheel_rear", 0, 3),
    LinkInfo("right_wheel_rear", 0, 3),
    LinkInfo("body", 0, 1),
    LinkInfo("front_lidar_link", 0, 1),
    LinkInfo("top_lidar_link", 0, 1),
    LinkInfo("rear_right_lidar_link", 0, 1),
    LinkInfo("rear_left_lidar_link", 0, 1),
    LinkInfo("chassis_floor", 1, 1),
    LinkInfo("front_axle", 1, 1),
    LinkInfo("left_tie_rod_arm", 1, 2),
    LinkInfo("left_hub", 1, 1),
    LinkInfo("tie_rod", 1, 1),
    LinkInfo("left_wheel", 1, 3),
    LinkInfo("right_tie_rod_arm", 1, 2),
    LinkInfo("right_hub", 1, 1),
    LinkInfo("right_wheel", 1, 3),
    LinkInfo("rear_axle", 1, 1),
    LinkInfo("left_wheel_rear", 1, 3),
    LinkInfo("right_wheel_rear", 1, 3),
    LinkInfo("body", 1, 1),
    LinkInfo("front_lidar_link", 1, 1),
    LinkInfo("top_lidar_link", 1, 1),
    LinkInfo("rear_right_lidar_link", 1, 1),
    LinkInfo("rear_left_lidar_link", 1, 1)
  };

  for (int i = 0; i < load_message.num_links; ++i) {
    EXPECT_EQ(load_message.link.at(i).name, expected_load.at(i).name);
    EXPECT_EQ(load_message.link.at(i).robot_num, expected_load.at(i).robot_num);
    EXPECT_EQ(load_message.link.at(i).num_geom, expected_load.at(i).num_geom);
  }

  // Verifies that the correct lcmt_viewer_draw message was transmitted. The
  // tolerance values were empirically determined.
  const lcmt_viewer_draw draw_message =
      mock_lcm->DecodeLastPublishedMessageAs<lcmt_viewer_draw>(
          "DRAKE_VIEWER_DRAW");
  EXPECT_EQ(draw_message.num_links, expected_num_links);

  // Checks the chassis_floor body of the first car.
  EXPECT_EQ(draw_message.link_name.at(0), "chassis_floor");
  EXPECT_EQ(draw_message.robot_num.at(0), 0);
  EXPECT_NEAR(draw_message.position.at(0).at(0),
      PriusVis<double>::kVisOffset + 0.99, 1e-6);
  EXPECT_NEAR(draw_message.position.at(0).at(1), 0, 1e-8);
  EXPECT_NEAR(draw_message.position.at(0).at(2), 0.378326, 1e-8);
  EXPECT_NEAR(draw_message.quaternion.at(0).at(0), 1, 1e-8);
  EXPECT_NEAR(draw_message.quaternion.at(0).at(1), 0, 1e-8);
  EXPECT_NEAR(draw_message.quaternion.at(0).at(2), 0, 1e-8);
  EXPECT_NEAR(draw_message.quaternion.at(0).at(3), 0, 1e-8);

  // Verifies that the first car is about 1 m ahead of the second car. This is
  // expected since the first car is traveling at 1 m/s for a second while the
  // second car hasn't started to move yet.
  const int n = draw_message.num_links / 2;
  for (int i = 0; i < n; ++i) {
    EXPECT_EQ(draw_message.link_name.at(i), draw_message.link_name.at(i + n));
    EXPECT_EQ(draw_message.robot_num.at(i),
              draw_message.robot_num.at(i + n) - 1);
    EXPECT_NEAR(draw_message.position.at(i).at(0),
                draw_message.position.at(i + n).at(0) + 0.99, 1e-6);
    EXPECT_NEAR(draw_message.position.at(i).at(1),
                draw_message.position.at(i + n).at(1), 1e-8);
    EXPECT_NEAR(draw_message.position.at(i).at(2),
                draw_message.position.at(i + n).at(2), 1e-8);
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(draw_message.quaternion.at(i).at(j),
                  draw_message.quaternion.at(i + n).at(j), 1e-8);
    }
  }
}

// Verifies that CarVisApplicator, PoseBundleToDrawMessage, and
// LcmPublisherSystem are instantiated in AutomotiveSimulator's Diagram and
// collectively result in the correct LCM messages being published.
GTEST_TEST(AutomotiveSimulatorTest, TestLcmOutput) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<lcm::DrakeMockLcm>());

  simulator->AddPriusSimpleCar("Model1", "Channel1");
  simulator->AddPriusSimpleCar("Model2", "Channel2");

  typedef Curve2<double> Curve2d;
  typedef Curve2d::Point2 Point2d;
  const std::vector<Point2d> waypoints{Point2d{0, 0}, Point2d{1, 0}};
  const Curve2d curve{waypoints};
  simulator->AddPriusTrajectoryCar(curve, 1 /* speed */, 0 /* start time */);
  simulator->AddPriusTrajectoryCar(curve, 1 /* speed */, 0 /* start time */);

  simulator->Start();
  simulator->StepBy(1e-3);

  const lcm::DrakeLcmInterface* lcm = simulator->get_lcm();
  ASSERT_NE(lcm, nullptr);

  const lcm::DrakeMockLcm* mock_lcm =
      dynamic_cast<const lcm::DrakeMockLcm*>(lcm);
  ASSERT_NE(mock_lcm, nullptr);

  const int expected_num_links = PriusVis<double>(0, "").num_poses() * 4;

  // Verifies that an lcmt_viewer_load_robot message was transmitted.
  const lcmt_viewer_load_robot load_message =
      mock_lcm->DecodeLastPublishedMessageAs<lcmt_viewer_load_robot>(
          "DRAKE_VIEWER_LOAD_ROBOT");
  EXPECT_EQ(load_message.num_links, expected_num_links);

  // Verifies that an lcmt_viewer_draw message was transmitted.
  const lcmt_viewer_draw draw_message =
      mock_lcm->DecodeLastPublishedMessageAs<lcmt_viewer_draw>(
          "DRAKE_VIEWER_DRAW");
  EXPECT_EQ(draw_message.num_links, expected_num_links);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
