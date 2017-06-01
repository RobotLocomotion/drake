#include "drake/automotive/automotive_simulator.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/automotive/curve2.h"
#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/prius_vis.h"
#include "drake/common/drake_path.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcmt_simple_car_state_t.hpp"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_context.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"
#include "drake/systems/rendering/pose_bundle.h"

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

  simulator->AddPriusSimpleCar("My Test Model", "Channel", initial_state);
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

GTEST_TEST(AutomotiveSimulatorTest, TestMobilControlledSimpleCar) {
  // TODO(jwnimmer-tri) Do something better than "0_" here.
  const std::string kJointStateChannelName = "0_FLOATING_JOINT_STATE";

  const std::string joint_state_name =
      systems::lcm::LcmPublisherSystem::make_name(kJointStateChannelName);

  // Set up a basic simulation with a MOBIL- and IDM-controlled SimpleCar.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<lcm::DrakeMockLcm>());
  lcm::DrakeMockLcm* lcm =
      dynamic_cast<lcm::DrakeMockLcm*>(simulator->get_lcm());
  ASSERT_NE(lcm, nullptr);

  const maliput::api::RoadGeometry* road{};
  EXPECT_NO_THROW(road = simulator->SetRoadGeometry(
      std::make_unique<const maliput::dragway::RoadGeometry>(
          maliput::api::RoadGeometryId({"TestDragway"}), 2 /* num lanes */,
          100 /* length */, 4 /* lane width */, 1 /* shoulder width */)));

  // Create one MOBIL car and two stopped cars arranged as follows:
  //
  // ---------------------------------------------------------------
  // ^  +r, +y                                          | Decoy 2 |
  // |    -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
  // +---->  +s, +x  | MOBIL Car |   | Decoy 1 |
  // ---------------------------------------------------------------
  SimpleCarState<double> simple_car_state;
  simple_car_state.set_x(2);
  simple_car_state.set_y(-2);
  simple_car_state.set_velocity(10);
  const int id_mobil =
      simulator->AddMobilControlledSimpleCar("mobil", true /* with_s */,
                                             simple_car_state);
  EXPECT_EQ(id_mobil, 0);

  MaliputRailcarState<double> decoy_state;
  decoy_state.set_s(6);
  decoy_state.set_speed(0);
  const int id_decoy1 = simulator->AddPriusMaliputRailcar(
      "decoy1", LaneDirection(road->junction(0)->segment(0)->lane(0)),
      MaliputRailcarParams<double>(), decoy_state);
  EXPECT_EQ(id_decoy1, 1);

  decoy_state.set_s(20);
  const int id_decoy2 = simulator->AddPriusMaliputRailcar(
      "decoy2", LaneDirection(road->junction(0)->segment(0)->lane(1)),
      MaliputRailcarParams<double>(), decoy_state);
  EXPECT_EQ(id_decoy2, 2);

  // Finish all initialization, so that we can test the post-init state.
  simulator->Start();

  // Advances the simulation to allow the MaliputRailcar to begin accelerating.
  simulator->StepBy(0.5);

  const lcmt_viewer_draw draw_message =
      lcm->DecodeLastPublishedMessageAs<lcmt_viewer_draw>("DRAKE_VIEWER_DRAW");
  EXPECT_EQ(draw_message.num_links, 3 * PriusVis<double>(0, "").num_poses());

  // Expect the SimpleCar to start steering to the left; y value increases.
  const double mobil_y = draw_message.position.at(0).at(1);
  EXPECT_GE(mobil_y, -2.);
}

// Cover AddTrajectoryCar (and thus AddPublisher).
GTEST_TEST(AutomotiveSimulatorTest, TestPriusTrajectoryCar) {
  typedef Curve2<double> Curve2d;
  typedef Curve2d::Point2 Point2d;
  const std::vector<Point2d> waypoints{
      {0.0, 0.0}, {100.0, 0.0},
  };
  const Curve2d curve{waypoints};

  // Set up a basic simulation with a couple Prius TrajectoryCars. Both cars
  // start at position zero; the first has a speed of 1 m/s, while the other is
  // stationary. They both follow a straight 100 m long line.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<lcm::DrakeMockLcm>());
  const int id1 = simulator->AddPriusTrajectoryCar("alice", curve, 1.0, 0.0);
  const int id2 = simulator->AddPriusTrajectoryCar("bob", curve, 0.0, 0.0);
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

  // Plus one to include the world.
  const int expected_num_links = PriusVis<double>(0, "").num_poses() * 2 + 1;

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

  const std::vector<LinkInfo> expected_load{
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
      LinkInfo("rear_left_lidar_link", 1, 1),
      LinkInfo("world", 0, 0)};

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
  // Minus one to omit world, which remains still.
  EXPECT_EQ(draw_message.num_links, expected_num_links - 1);

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
  // second car is immobile.
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

// Returns the x-position of the vehicle based on an lcmt_viewer_draw message.
// It also checks that the y-position of the vehicle is equal to the provided y
// value.
double GetPosition(const lcmt_viewer_draw& message, double y) {
  EXPECT_EQ(message.num_links, PriusVis<double>(0, "").num_poses());
  EXPECT_EQ(message.link_name.at(0), "chassis_floor");
  EXPECT_DOUBLE_EQ(message.position.at(0).at(1), y);
  return message.position.at(0).at(0);
}

// Covers AddMaliputRailcar().
GTEST_TEST(AutomotiveSimulatorTest, TestMaliputRailcar) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<lcm::DrakeMockLcm>());
  lcm::DrakeMockLcm* lcm =
      dynamic_cast<lcm::DrakeMockLcm*>(simulator->get_lcm());
  ASSERT_NE(lcm, nullptr);
  const double kR{0.5};
  MaliputRailcarParams<double> params;
  params.set_r(kR);

  EXPECT_THROW(simulator->AddPriusMaliputRailcar("foo", LaneDirection()),
               std::runtime_error);

  const maliput::api::RoadGeometry* road{};
  EXPECT_NO_THROW(
      road = simulator->SetRoadGeometry(
          std::make_unique<const maliput::dragway::RoadGeometry>(
              maliput::api::RoadGeometryId({"TestDragway"}), 1 /* num lanes */,
              100 /* length */, 4 /* lane width */, 1 /* shoulder width */)));

  EXPECT_THROW(
      simulator->AddPriusMaliputRailcar("bar", LaneDirection(), params),
      std::runtime_error);

  const auto different_road =
      std::make_unique<const maliput::dragway::RoadGeometry>(
          maliput::api::RoadGeometryId({"DifferentDragway"}), 2 /* num lanes */,
          50 /* length */, 3 /* lane width */, 2 /* shoulder width */);

  EXPECT_THROW(simulator->AddPriusMaliputRailcar(
                   "bar", LaneDirection(
                              different_road->junction(0)->segment(0)->lane(0)),
                   params),
               std::runtime_error);

  const int id = simulator->AddPriusMaliputRailcar(
      "model_name", LaneDirection(road->junction(0)->segment(0)->lane(0)),
      params, MaliputRailcarState<double>() /* initial state */);
  EXPECT_EQ(id, 0);

  simulator->Start();

  // Takes two steps to trigger the publishing of an LCM draw message.
  simulator->StepBy(0.005);
  simulator->StepBy(0.005);

  const double initial_x = PriusVis<double>::kVisOffset;

  // Verifies the acceleration is zero even if
  // AutomotiveSimulator::SetMaliputRailcarAccelerationCommand() was not called.
  const lcmt_viewer_draw draw_message0 =
      lcm->DecodeLastPublishedMessageAs<lcmt_viewer_draw>("DRAKE_VIEWER_DRAW");
  // The following tolerance was determined empirically.
  EXPECT_NEAR(GetPosition(draw_message0, kR), initial_x, 1e-4);

  // Sets the commanded acceleration to be zero.
  simulator->SetMaliputRailcarAccelerationCommand(id, 0);
  simulator->StepBy(0.005);
  simulator->StepBy(0.005);

  // Verifies that the vehicle hasn't moved yet. This is expected since the
  // commanded acceleration is zero.
  const lcmt_viewer_draw draw_message1 =
      lcm->DecodeLastPublishedMessageAs<lcmt_viewer_draw>("DRAKE_VIEWER_DRAW");
  // The following tolerance was determined empirically.
  EXPECT_NEAR(GetPosition(draw_message1, kR), initial_x, 1e-4);

  // Sets the commanded acceleration to be 10 m/s^2.
  simulator->SetMaliputRailcarAccelerationCommand(id, 10);

  // Advances the simulation to allow the MaliputRailcar to begin accelerating.
  simulator->StepBy(0.005);
  simulator->StepBy(0.005);

  // Verifies that the MaliputRailcar has moved forward relative to prior to
  // the nonzero acceleration command being issued.
  const lcmt_viewer_draw draw_message2 =
      lcm->DecodeLastPublishedMessageAs<lcmt_viewer_draw>("DRAKE_VIEWER_DRAW");
  EXPECT_LT(draw_message1.position.at(0).at(0), GetPosition(draw_message2, kR));
}

bool ContainsWorld(const lcmt_viewer_load_robot& message) {
  bool result = false;
  for (int i = 0; i < message.num_links; ++i) {
    if (message.link.at(i).name ==
        std::string(RigidBodyTreeConstants::kWorldName)) {
      result = true;
    }
  }
  return result;
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
  simulator->AddPriusTrajectoryCar("alice", curve, 1 /* speed */,
                                   0 /* start time */);
  simulator->AddPriusTrajectoryCar("bob", curve, 1 /* speed */,
                                   0 /* start time */);

  simulator->Start();
  simulator->StepBy(1e-3);

  const lcm::DrakeLcmInterface* lcm = simulator->get_lcm();
  ASSERT_NE(lcm, nullptr);

  const lcm::DrakeMockLcm* mock_lcm =
      dynamic_cast<const lcm::DrakeMockLcm*>(lcm);
  ASSERT_NE(mock_lcm, nullptr);

  // Plus one to include the world.
  const int expected_num_links = PriusVis<double>(0, "").num_poses() * 4 + 1;

  // Verifies that an lcmt_viewer_load_robot message was transmitted.
  const lcmt_viewer_load_robot load_message =
      mock_lcm->DecodeLastPublishedMessageAs<lcmt_viewer_load_robot>(
          "DRAKE_VIEWER_LOAD_ROBOT");
  EXPECT_EQ(load_message.num_links, expected_num_links);
  EXPECT_TRUE(ContainsWorld(load_message));

  // Verifies that an lcmt_viewer_draw message was transmitted.
  const lcmt_viewer_draw draw_message =
      mock_lcm->DecodeLastPublishedMessageAs<lcmt_viewer_draw>(
          "DRAKE_VIEWER_DRAW");
  // Minus one to omit world, which remains still.
  EXPECT_EQ(draw_message.num_links, expected_num_links - 1);
}

// Verifies that exceptions are thrown if a vehicle with a non-unique name is
// added to the simulation.
GTEST_TEST(AutomotiveSimulatorTest, TestDuplicateVehicleNameException) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<lcm::DrakeMockLcm>());

  EXPECT_NO_THROW(simulator->AddPriusSimpleCar("Model1", "Channel1"));
  EXPECT_THROW(simulator->AddPriusSimpleCar("Model1", "foo"),
               std::runtime_error);

  typedef Curve2<double> Curve2d;
  typedef Curve2d::Point2 Point2d;
  const std::vector<Point2d> waypoints{Point2d{0, 0}, Point2d{1, 0}};
  const Curve2d curve{waypoints};

  EXPECT_NO_THROW(simulator->AddPriusTrajectoryCar(
      "alice", curve, 1 /* speed */, 0 /* start time */));
  EXPECT_THROW(simulator->AddPriusTrajectoryCar("alice", curve, 1 /* speed */,
                                                0 /* start time */),
               std::runtime_error);
  EXPECT_THROW(simulator->AddPriusTrajectoryCar("Model1", curve, 1 /* speed */,
                                                0 /* start time */),
               std::runtime_error);

  const MaliputRailcarParams<double> params;
  const maliput::api::RoadGeometry* road{};
  EXPECT_NO_THROW(
      road = simulator->SetRoadGeometry(
          std::make_unique<const maliput::dragway::RoadGeometry>(
              maliput::api::RoadGeometryId({"TestDragway"}), 1 /* num lanes */,
              100 /* length */, 4 /* lane width */, 1 /* shoulder width */)));
  EXPECT_NO_THROW(simulator->AddPriusMaliputRailcar(
      "Foo", LaneDirection(road->junction(0)->segment(0)->lane(0)), params,
      MaliputRailcarState<double>() /* initial state */));
  EXPECT_THROW(
      simulator->AddPriusMaliputRailcar(
          "alice", LaneDirection(road->junction(0)->segment(0)->lane(0)),
          params, MaliputRailcarState<double>() /* initial state */),
      std::runtime_error);
  EXPECT_THROW(
      simulator->AddPriusMaliputRailcar(
          "Model1", LaneDirection(road->junction(0)->segment(0)->lane(0)),
          params, MaliputRailcarState<double>() /* initial state */),
      std::runtime_error);
}

// Verifies that no exception is thrown when multiple IDM-controlled
// MaliputRailcar vehicles are simulated. This prevents a regression of #5886.
GTEST_TEST(AutomotiveSimulatorTest, TestIdmControllerUniqueName) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<lcm::DrakeMockLcm>());

  const MaliputRailcarParams<double> params;
  const maliput::api::RoadGeometry* road = simulator->SetRoadGeometry(
      std::make_unique<const maliput::dragway::RoadGeometry>(
          maliput::api::RoadGeometryId({"TestDragway"}), 1 /* num lanes */,
          100 /* length */, 4 /* lane width */, 1 /* shoulder width */));
  simulator->AddIdmControlledPriusMaliputRailcar(
      "Alice", LaneDirection(road->junction(0)->segment(0)->lane(0)), params,
      MaliputRailcarState<double>() /* initial state */);
  simulator->AddIdmControlledPriusMaliputRailcar(
      "Bob", LaneDirection(road->junction(0)->segment(0)->lane(0)), params,
      MaliputRailcarState<double>() /* initial state */);

  EXPECT_NO_THROW(simulator->Start());
}

// Verifies that the velocity outputs of the MaliputRailcars are connected to
// the PoseAggregator, which prevents a regression of #5894.
GTEST_TEST(AutomotiveSimulatorTest, TestRailcarVelocityOutput) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<lcm::DrakeMockLcm>());

  const MaliputRailcarParams<double> params;
  const maliput::api::RoadGeometry* road =
      simulator->SetRoadGeometry(
          std::make_unique<const maliput::dragway::RoadGeometry>(
              maliput::api::RoadGeometryId({"TestDragway"}), 1 /* num lanes */,
              100 /* length */, 4 /* lane width */, 1 /* shoulder width */));
  MaliputRailcarState<double> alice_initial_state;
  alice_initial_state.set_s(5);
  alice_initial_state.set_speed(1);
  const int alice_id = simulator->AddPriusMaliputRailcar("Alice",
      LaneDirection(road->junction(0)->segment(0)->lane(0)), params,
      alice_initial_state);
  const int bob_id = simulator->AddIdmControlledPriusMaliputRailcar("Bob",
      LaneDirection(road->junction(0)->segment(0)->lane(0)), params,
      MaliputRailcarState<double>() /* initial state */);

  EXPECT_NO_THROW(simulator->Start());

  // Advances the simulation to allow Alice's MaliputRailcar to move at fixed
  // speed and Bob's MaliputRailcar to move under IDM control.
  simulator->StepBy(1);

  const int kAliceIndex{0};
  const int kBobIndex{1};

  // Verifies that the velocity within the PoseAggregator's PoseBundle output is
  // non-zero.
  const systems::rendering::PoseBundle<double> poses =
      simulator->GetCurrentPoses();
  ASSERT_EQ(poses.get_num_poses(), 2);
  ASSERT_EQ(poses.get_model_instance_id(kAliceIndex), alice_id);
  ASSERT_EQ(poses.get_model_instance_id(kBobIndex), bob_id);
  EXPECT_FALSE(poses.get_velocity(kAliceIndex).get_value().isZero());
  EXPECT_FALSE(poses.get_velocity(kBobIndex).get_value().isZero());
}

// Tests Build/Start logic
GTEST_TEST(AutomotiveSimulatorTest, TestBuild) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  simulator->AddPriusSimpleCar("Model1", "Channel1");
  simulator->AddPriusSimpleCar("Model2", "Channel2");

  simulator->Build();
  EXPECT_FALSE(simulator->has_started());
  EXPECT_NO_THROW(simulator->GetDiagram());

  simulator->Start(0.0);
  EXPECT_TRUE(simulator->has_started());
  EXPECT_NO_THROW(simulator->GetDiagram());
}

// Tests Build/Start logic (calling Start only)
GTEST_TEST(AutomotiveSimulatorTest, TestBuild2) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  simulator->AddPriusSimpleCar("Model1", "Channel1");
  simulator->AddPriusSimpleCar("Model2", "Channel2");

  simulator->Start(0.0);
  EXPECT_NO_THROW(simulator->GetDiagram());
}

}  // namespace
}  // namespace automotive
}  // namespace drake
