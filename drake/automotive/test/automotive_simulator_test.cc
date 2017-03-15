#include "drake/automotive/automotive_simulator.h"

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_simple_car_state_t.hpp"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
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

// A helper method for unit testing SimpleCar. Parameter @p sdf_filename is the
// name of the file containing the model of the vehicle to be used by the
// SimpleCar.
void TestSimpleCarWithSdf(const std::string& sdf_filename,
    int num_vehicle_bodies) {
  // TODO(jwnimmer-tri) Do something better than "0_" here.
  const std::string kJointStateChannelName = "0_FLOATING_JOINT_STATE";
  const std::string kCommandChannelName = "DRIVING_COMMAND";

  const std::string driving_command_name =
      systems::lcm::LcmSubscriberSystem::make_name(kCommandChannelName);
  const std::string joint_state_name =
      systems::lcm::LcmPublisherSystem::make_name(kJointStateChannelName);

  // Set up a basic simulation with just SimpleCar and its hangers-on.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<lcm::DrakeMockLcm>());
  const int model_instance_id =
      simulator->AddSimpleCarFromSdf(sdf_filename, "", kCommandChannelName);

  // Obtain the number of bodies belonging to the model.
  const int num_vehicle_bodies_in_tree =
      simulator->get_rigid_body_tree()
          .FindModelInstanceBodies(model_instance_id).size();
  EXPECT_EQ(num_vehicle_bodies_in_tree, num_vehicle_bodies);

  // Grab the systems we want while testing GetBuilderSystemByName() in the
  // process.
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
  ASSERT_EQ(1 + num_vehicle_bodies, tree.get_num_bodies());

  // Get the rigid bodies belonging to the vehicle's model instance.
  const std::vector<const RigidBody<double>*> vehicle_bodies =
      tree.FindModelInstanceBodies(model_instance_id);
  EXPECT_EQ(static_cast<int>(vehicle_bodies.size()), num_vehicle_bodies);

  const auto& body = tree.get_body(1);
  EXPECT_EQ(vehicle_bodies.at(0)->get_name(), body.get_name());

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

  // One body belongs to the world, the rest belong to the car model.
  EXPECT_EQ(published_draw_message.num_links, 1 + num_vehicle_bodies);
  EXPECT_EQ(published_draw_message.link_name.at(0), "world");
  EXPECT_EQ(published_draw_message.link_name.at(1),
            vehicle_bodies.at(0)->get_name());

  // The subsystem pointers must not change.
  EXPECT_EQ(&simulator->GetDiagramSystemByName(driving_command_name),
            &command_sub);
  EXPECT_EQ(&simulator->GetDiagramSystemByName(joint_state_name), &state_pub);
}

// Cover AddSimpleCar (and thus AddPublisher), Start, StepBy, GetSystemByName.
GTEST_TEST(AutomotiveSimulatorTest, SimpleCarTestPriusWithLidar) {
  TestSimpleCarWithSdf(GetDrakePath() +
                       "/automotive/models/prius/prius_with_lidar.sdf", 17);
}

GTEST_TEST(AutomotiveSimulatorTest, SimpleCarTestPrius) {
  TestSimpleCarWithSdf(GetDrakePath() +
                       "/automotive/models/prius/prius.sdf", 13);
}

// Tests the ability to initialize a SimpleCar to a non-zero initial state.
GTEST_TEST(AutomotiveSimulatorTest, TestSimpleCarInitialState) {
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

  simulator->AddSimpleCarFromSdf(
      GetDrakePath() + "/automotive/models/prius/prius_with_lidar.sdf",
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

// A helper method for unit testing TrajectoryCar. Parameters @p sdf_filename_1
// and @p sdf_filename_2 are the names of the files containing the models of the
// vehicles to be used by two TrajectoryCars.
void TestTrajectoryCarWithSdf(const std::string& sdf_file_1, int num_bodies_1,
                              const std::string& sdf_file_2, int num_bodies_2) {
  typedef Curve2<double> Curve2d;
  typedef Curve2d::Point2 Point2d;
  const std::vector<Point2d> waypoints{
      {0.0, 0.0},
      {100.0, 0.0},
  };
  const Curve2d curve{waypoints};

  // Set up a basic simulation with just some TrajectoryCars.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<lcm::DrakeMockLcm>());
  const int model_instance_id_1 =
      simulator->AddTrajectoryCarFromSdf(sdf_file_1, curve, 1.0, 0.0);
  const int model_instance_id_2 =
      simulator->AddTrajectoryCarFromSdf(sdf_file_2, curve, 1.0, 10.0);

  // Obtain the number of bodies in the models.
  const std::vector<const RigidBody<double>*> vehicle_bodies_1 =
      simulator->get_rigid_body_tree().FindModelInstanceBodies(
          model_instance_id_1);
  const std::vector<const RigidBody<double>*> vehicle_bodies_2 =
      simulator->get_rigid_body_tree().FindModelInstanceBodies(
          model_instance_id_2);
  EXPECT_EQ(static_cast<int>(vehicle_bodies_1.size()), num_bodies_1);
  EXPECT_EQ(static_cast<int>(vehicle_bodies_2.size()), num_bodies_2);

  // Finish all initialization, so that we can test the post-init state.
  simulator->Start();

  // Confirm that the RigidBodyTree has been appropriately amended.
  const auto& tree = simulator->get_rigid_body_tree();
  EXPECT_EQ(2, tree.get_num_model_instances());
  // One body belongs to the world, the rest belong to two car models.
  ASSERT_EQ(1 + num_bodies_1 + num_bodies_2, tree.get_num_bodies());

  // Verifies that the first car was added to the tree.
  EXPECT_EQ(vehicle_bodies_1.at(0)->get_name(), tree.get_body(1).get_name());

  // Verifies that the second car was added to the tree.
  EXPECT_EQ(vehicle_bodies_2.at(0)->get_name(),
            tree.get_body(num_bodies_1 + 1).get_name());

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

// Cover AddTrajectoryCar (and thus AddPublisher).
GTEST_TEST(AutomotiveSimulatorTest, TrajectoryCarTestTwoDofBot) {
  TestTrajectoryCarWithSdf(GetDrakePath() +
                           "/automotive/models/prius/prius.sdf", 13,
                           GetDrakePath() +
                           "/automotive/models/prius/prius_with_lidar.sdf", 17);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
