#include "drake/automotive/mobil_planner.h"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/test/idm_test.h"

namespace drake {
namespace automotive {
namespace {

using maliput::api::Lane;
using maliput::dragway::RoadGeometry;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;
using systems::rendering::PoseBundle;

constexpr int kNumLanes{2};
constexpr double kEgoXPosition{10.};
constexpr double kLaneWidth{4.};
constexpr double kEgoSpeed{10.};

constexpr int kIndexRightCar{0};
constexpr int kIndexLeftCar{1};
constexpr int kIndexEgo{2};

constexpr int kRightLaneIndex{0};
constexpr int kLeftLaneIndex{1};

class MobilPlannerTest : public IdmTest {
 protected:
  void Initialize() override {
    // Create a straight road with two lanes starting at `x = 0` and heading
    // along the positive-x axis of the world frame, with Lane 0 at `y =
    // -kLaneWidth / 2`, and Lane 1 at `y = kLaneWidth / 2`.
    road_.reset(new maliput::dragway::RoadGeometry(
        maliput::api::RoadGeometryId({"Two-Lane Dragway"}),
        kNumLanes /* num_lanes */, 100 /* length */,
        kLaneWidth /* lane_width */, 0. /* shoulder_width */));
    segment_ = road_->junction(0)->segment(0);
    ExtractLaneDirectionsFromDragway();
  }

  // Initializes MobilPlanner with the dragway at a given LaneDirection.
  void InitializeMobilPlanner(const LaneDirection& initial_lane_direction) {
    dut_.reset(new MobilPlanner<double>(*road_, initial_lane_direction));
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);

    mp_ = dynamic_cast<const MobilPlanner<double>*>(dut_.get());
    DRAKE_DEMAND(mp_ != nullptr);
    ego_pose_input_index_ = mp_->ego_pose_input().get_index();
    ego_velocity_input_index_ = mp_->ego_velocity_input().get_index();
    traffic_input_index_ = mp_->traffic_input().get_index();
  }

  const MobilPlanner<double>* mp_{nullptr};
  std::unique_ptr<maliput::api::RoadGeometry> road_;
  const maliput::api::Segment* segment_;
  std::vector<LaneDirection> lane_directions_{};

  void ExtractLaneDirectionsFromDragway() {
    DRAKE_DEMAND(lane_directions_.empty());
    for (int i = 0; i < segment_->num_lanes(); ++i) {
      lane_directions_.push_back({segment_->lane(i), true});
    }
  }

  int get_lane_index(const Lane* lane) {
    if (segment_->lane(0)->id().id == lane->id().id) {
      return 0;
    } else if (segment_->lane(1)->id().id == lane->id().id) {
      return 1;
    } else {
      throw std::runtime_error("Lane does not belong to road_.");
    }
  }

  // Creates poses for one ego car and two traffic cars.
  void SetDefaultMultiLanePoses(const LaneDirection& initial_lane_direction,
                                const double delta_position_right_car,
                                const double delta_position_left_car) {
    const int lane_index = get_lane_index(initial_lane_direction.lane);
    auto ego_pose = std::make_unique<PoseVector<double>>();
    auto ego_velocity = std::make_unique<FrameVelocity<double>>();
    PoseBundle<double> traffic_poses(3);  // Bundles the ego and traffic cars.

    // Configure the ego car pose and velocity.
    const Eigen::Translation3d translation_ego(
        kEgoXPosition /* x */, kLaneWidth * (lane_index - 0.5) /* y */,
        0. /* z */);
    ego_pose->set_translation(translation_ego);
    context_->FixInputPort(ego_pose_input_index_, std::move(ego_pose));

    Vector6<double> velocity{};
    velocity << 0. /* ωx */, 0. /* ωy */, 0. /* ωz */, kEgoSpeed /* vx */,
        0. /* vy */, 0. /* vz */;
    ego_velocity->set_velocity(multibody::SpatialVelocity<double>(velocity));
    context_->FixInputPort(ego_velocity_input_index_, std::move(ego_velocity));

    // Configure the traffic poses and velocities, inclusive of the ego car,
    // where all cars are traveling at the same x-velocity.
    FrameVelocity<double> all_velocity;
    all_velocity.set_velocity(multibody::SpatialVelocity<double>(velocity));
    const Eigen::Translation3d translation_right(
        kEgoXPosition + delta_position_right_car /* x */,
        -kLaneWidth / 2. /* y */, 0. /* z */);
    const Eigen::Translation3d translation_left(
        kEgoXPosition + delta_position_left_car /* x */,
        kLaneWidth / 2. /* y */, 0. /* z */);
    traffic_poses.set_pose(kIndexRightCar,
                           Eigen::Isometry3d(translation_right));
    traffic_poses.set_velocity(kIndexRightCar, all_velocity);
    traffic_poses.set_pose(kIndexLeftCar, Eigen::Isometry3d(translation_left));
    traffic_poses.set_velocity(kIndexLeftCar, all_velocity);
    traffic_poses.set_pose(kIndexEgo, Eigen::Isometry3d(translation_ego));
    traffic_poses.set_velocity(kIndexEgo, all_velocity);
    context_->FixInputPort(traffic_input_index_,
                           systems::AbstractValue::Make(traffic_poses));
  }
};

TEST_F(MobilPlannerTest, Topology) {
  const LaneDirection initial_lane_direction =
      LaneDirection(segment_->lane(kRightLaneIndex), true);
  InitializeMobilPlanner(initial_lane_direction);

  ASSERT_EQ(3, mp_->get_num_input_ports());
  const auto& ego_pose_input_descriptor =
      mp_->get_input_port(ego_pose_input_index_);
  EXPECT_EQ(systems::kVectorValued, ego_pose_input_descriptor.get_data_type());
  EXPECT_EQ(7 /* PoseVector input */, ego_pose_input_descriptor.size());
  const auto& ego_velocity_input_descriptor =
      mp_->get_input_port(ego_velocity_input_index_);
  EXPECT_EQ(systems::kVectorValued,
            ego_velocity_input_descriptor.get_data_type());
  EXPECT_EQ(6 /* FrameVelocity input */, ego_velocity_input_descriptor.size());
  const auto& traffic_input_descriptor =
      mp_->get_input_port(traffic_input_index_);
  EXPECT_EQ(systems::kAbstractValued, traffic_input_descriptor.get_data_type());

  ASSERT_EQ(2, mp_->get_num_output_ports());
  const auto& command_output_descriptor =
      mp_->get_output_port(mp_->driving_command_output().get_index());
  EXPECT_EQ(systems::kVectorValued, command_output_descriptor.get_data_type());
  EXPECT_EQ(2 /* DrivingCommand output */, command_output_descriptor.size());
  const auto& lane_output_descriptor =
      mp_->get_output_port(mp_->lane_output().get_index());
  EXPECT_EQ(systems::kAbstractValued, lane_output_descriptor.get_data_type());
}

// Tests that the IDM equations are performing as expected.
TEST_F(MobilPlannerTest, IdmOutput) {
  // Arrange the ego car in the right lane and two cars ahead in the same lane.
  const LaneDirection initial_lane_direction =
      LaneDirection(segment_->lane(kRightLaneIndex), true);
  InitializeMobilPlanner(initial_lane_direction);

  // Define a pointer to where the DrivingCommand results end up.
  const auto result =
      output_->get_vector_data(mp_->driving_command_output().get_index());
  const auto command = dynamic_cast<const DrivingCommand<double>*>(result);
  ASSERT_NE(nullptr, command);

  this->TestIdmPlanner(*command);
}

// Tests the incentive of the ego car to change lanes when tailgating a car
// close ahead.
TEST_F(MobilPlannerTest, IncentiveWhileTailgating) {
  // Arrange the ego car in the right lane.
  InitializeMobilPlanner(lane_directions_[kRightLaneIndex]);

  // Define a pointer to where the DrivingCommand results end up.
  const auto result = output_->GetMutableData(mp_->lane_output().get_index());

  // Arrange two traffic cars ahead of the ego, with the right car closer ahead.
  SetDefaultMultiLanePoses(lane_directions_[kRightLaneIndex], /* ego lane */
                           5.,   /* right car position */
                           40.); /* left car position */

  mp_->CalcOutput(*context_, output_.get());
  auto lane_direction = result->template GetMutableValue<LaneDirection>();

  // Expect the left lane to be more desirable.
  EXPECT_EQ(lane_directions_[kLeftLaneIndex].lane->id().id,
            lane_direction.lane->id().id);
}

// Tests the incentive of the ego car to keep its current lane when a car is far
// ahead.
TEST_F(MobilPlannerTest, IncentiveWhileNotTailgating) {
  // Arrange the ego car in the left lane.
  InitializeMobilPlanner(lane_directions_[kLeftLaneIndex]);

  // Define a pointer to where the DrivingCommand results end up.
  const auto result = output_->GetMutableData(mp_->lane_output().get_index());

  // Arrange two traffic cars ahead of the ego, with the right car closer ahead.
  SetDefaultMultiLanePoses(lane_directions_[kLeftLaneIndex], /* ego lane */
                           5.,   /* right car position */
                           40.); /* left car position */

  mp_->CalcOutput(*context_, output_.get());
  auto lane_direction = result->template GetMutableValue<LaneDirection>();

  // Expect the left lane to be more desirable.
  EXPECT_EQ(lane_directions_[kLeftLaneIndex].lane->id().id,
            lane_direction.lane->id().id);
}

// Tests the politeness of the ego car to keep its current lane when a car is
// far behind.
TEST_F(MobilPlannerTest, PolitenessNoTailgator) {
  // Arrange the ego car in the right lane.
  InitializeMobilPlanner(lane_directions_[kRightLaneIndex]);

  // Define a pointer to where the DrivingCommand results end up.
  const auto result = output_->GetMutableData(mp_->lane_output().get_index());

  // Arrange two traffic cars behind the ego, with the left car closer behind.
  SetDefaultMultiLanePoses(lane_directions_[kRightLaneIndex], /* ego lane */
                           -40., /* right car position */
                           -5.); /* left car position */

  mp_->CalcOutput(*context_, output_.get());
  auto lane_direction = result->template GetMutableValue<LaneDirection>();

  // Expect the right lane to be more desirable.
  EXPECT_EQ(lane_directions_[kRightLaneIndex].lane->id().id,
            lane_direction.lane->id().id);
}

// Tests the politeness of the ego car to chage lanes when being tailgated by a
// car close behind.
TEST_F(MobilPlannerTest, PolitenessWithTailgator) {
  // Arrange the ego car in the right lane.
  InitializeMobilPlanner(lane_directions_[kRightLaneIndex]);

  // Define a pointer to where the DrivingCommand results end up.
  const auto result = output_->GetMutableData(mp_->lane_output().get_index());

  // Arrange two traffic cars behind the ego, with the right car closer behind.
  SetDefaultMultiLanePoses(lane_directions_[kRightLaneIndex], /* ego lane */
                           -5.,   /* right car position */
                           -40.); /* left car position */

  mp_->CalcOutput(*context_, output_.get());
  auto lane_direction = result->template GetMutableValue<LaneDirection>();

  // Expect the left lane to be more desirable.
  EXPECT_EQ(lane_directions_[kLeftLaneIndex].lane->id().id,
            lane_direction.lane->id().id);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
