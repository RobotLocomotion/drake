#include "drake/automotive/automotive_simulator.h"

#include "gtest/gtest.h"

#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"

namespace drake {
namespace automotive {
namespace {

const int kNumVehicleBodies = 17;  // The number of bodies in the car model.

// Simple touches on the getters.
GTEST_TEST(AutomotiveSimulatorTest, BasicTest) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  EXPECT_NE(nullptr, simulator->get_lcm());
  EXPECT_NE(nullptr, simulator->get_builder());
}

// Obtains the serialized version of the last message transmitted on LCM channel
// @p channel. Uses @p translator to decode the message into @p joint_value.
void GetLastPublishedJointValue(const std::string& channel,
    const systems::lcm::LcmAndVectorBaseTranslator& translator,
    lcm::DrakeMockLcm* mock_lcm, EulerFloatingJointState<double>* joint_value) {
  const std::vector<uint8_t>& message =
      mock_lcm->get_last_published_message(channel);
  translator.Deserialize(message.data(), message.size(), joint_value);
}

// TODO(liang.fok): When AutomotiveSimulator's API is updated to allow the model
// of SimpleCar to be client-selectable, be sure to avoid duplicating code. This
// can be done by generalizing the code in the following unit test and placing
// it in a method that can be called by numerous unit tests, each evaluating
// SimpleCar using a different model.
//
// Cover AddSimpleCar (and thus AddPublisher), Start, StepBy, GetSystemByName.
GTEST_TEST(AutomotiveSimulatorTest, SimpleCarTest) {
  const std::string kJointStateChannelName = "0_FLOATING_JOINT_STATE";
  const std::string kCommandChannelName = "DRIVING_COMMAND";

  const std::string driving_command_name =
      systems::lcm::LcmSubscriberSystem::get_name(kCommandChannelName);
  // TODO(jwnimmer-tri) Do something better than "0_" here.
  const std::string joint_state_name =
      systems::lcm::LcmPublisherSystem::get_name(kJointStateChannelName);

  // Set up a basic simulation with just SimpleCar and its hangers-on.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<lcm::DrakeMockLcm>());
  simulator->AddSimpleCar();
  // Grab the pieces we want (testing the GetSystemByName in the process).
  auto& command_sub = dynamic_cast<systems::lcm::LcmSubscriberSystem&>(
      simulator->GetBuilderSystemByName(driving_command_name));
  auto& state_pub = dynamic_cast<systems::lcm::LcmPublisherSystem&>(
      simulator->GetBuilderSystemByName(joint_state_name));

  // Finish all initialization, so that we can test the post-init state.
  simulator->Start();

  // Confirm that the RigidBodyTree has been appropriately amended.
  const auto& tree = simulator->get_rigid_body_tree();
  EXPECT_EQ(1, tree.get_num_model_instances());
  // One body belongs to the world, the rest belong to the car.
  ASSERT_EQ(1 + kNumVehicleBodies, tree.get_num_bodies());

  const auto& body = tree.get_body(1);
  EXPECT_EQ("chassis_floor", body.get_name());

  // Set full throttle.
  DrivingCommand<double> command;
  command.set_throttle(1.0);
  lcm::DrakeMockLcm* mock_lcm =
      dynamic_cast<lcm::DrakeMockLcm*>(simulator->get_lcm());
  ASSERT_NE(nullptr, mock_lcm);
  std::vector<uint8_t> message_bytes;
  command_sub.get_translator().Serialize(0.0 /* time */, command,
                                         &message_bytes);
  mock_lcm->InduceSubscriberCallback(kCommandChannelName, &message_bytes[0],
                                     message_bytes.size());

  // Shortly after starting, we should have not have moved much.
  simulator->StepBy(0.01);
  EulerFloatingJointState<double> joint_value;
  GetLastPublishedJointValue(kJointStateChannelName, state_pub.get_translator(),
                             mock_lcm, &joint_value);
  EXPECT_GT(joint_value.x(), 0.0);
  EXPECT_LT(joint_value.x(), 0.001);

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
  const std::vector<uint8_t>& published_message_bytes =
      mock_lcm->get_last_published_message("DRAKE_VIEWER_DRAW");

  drake::lcmt_viewer_draw published_draw_message;
  EXPECT_GT(published_draw_message.decode(&published_message_bytes[0], 0,
      published_message_bytes.size()), 0);

  // One body belongs to the world, the rest belong to the car model.
  EXPECT_EQ(published_draw_message.num_links, 1 + kNumVehicleBodies);
  EXPECT_EQ(published_draw_message.link_name.at(0), "world");
  EXPECT_EQ(published_draw_message.link_name.at(1), "chassis_floor");

  // The subsystem pointers must not change.
  EXPECT_EQ(
      &simulator->GetDiagramSystemByName(driving_command_name), &command_sub);
  EXPECT_EQ(&simulator->GetDiagramSystemByName(joint_state_name), &state_pub);
}

// Cover AddTrajectoryCar (and thus AddPublisher).
GTEST_TEST(AutomotiveSimulatorTest, TrajectoryCarTest) {
  typedef Curve2<double> Curve2d;
  typedef Curve2d::Point2 Point2d;
  const std::vector<Point2d> waypoints{
    {0.0, 0.0},
    {100.0, 0.0},
  };
  const Curve2d curve{waypoints};

  // Set up a basic simulation with just some TrajectoryCars.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  simulator->AddTrajectoryCar(curve, 1.0, 0.0);
  simulator->AddTrajectoryCar(curve, 1.0, 10.0);

  // Finish all initialization, so that we can test the post-init state.
  simulator->Start();

  // Confirm that the RigidBodyTree has been appropriately amended.
  const auto& tree = simulator->get_rigid_body_tree();
  EXPECT_EQ(2, tree.get_num_model_instances());
  // One body belongs to the world, the rest belong to two car models.
  ASSERT_EQ(1 + 2 * kNumVehicleBodies, tree.get_num_bodies());

  // Verifies that the first car was added to the tree.
  EXPECT_EQ("chassis_floor", tree.get_body(1).get_name());
  EXPECT_EQ("front_axle", tree.get_body(2).get_name());


  // Verifies that the second car was added to the tree.
  EXPECT_EQ("chassis_floor", tree.get_body(kNumVehicleBodies + 1).get_name());
  EXPECT_EQ("front_axle", tree.get_body(kNumVehicleBodies + 2).get_name());

  // Run for a while.
  for (int i = 0; i < 100; ++i) {
    simulator->StepBy(0.01);
  }

  // TODO(jwnimmer-tri) Confirm that appropriate draw messages are coming out.
  // In particular, ensure that the link values for each body are consistent
  // with the trajectory car -- that the indices haven't been messed up.
  // Let's wait until we have LCM mocking (#3546) before doing this.

  // No aborts is good enough.
  EXPECT_TRUE(true);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
