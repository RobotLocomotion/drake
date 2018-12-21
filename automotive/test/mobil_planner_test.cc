#include "drake/automotive/mobil_planner.h"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace automotive {
namespace {

using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::RoadPosition;
using maliput::dragway::RoadGeometry;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

constexpr double kEgoXPosition{10.};            // meters
constexpr double kLaneWidth{4.};                // meters
constexpr double kEgoSpeed{10.};                // m/s
constexpr double kEgoAccelerationCommand{-1.};  // m/s^2

class MobilPlannerTest : public ::testing::TestWithParam<RoadPositionStrategy> {
 protected:
  void InitializeDragway(int num_lanes) {
    DRAKE_ASSERT(num_lanes >= 0);
    // Create a dragway with the specified number of lanes starting at `x = 0`
    // and centered at `y = 0`.
    road_.reset(new maliput::dragway::RoadGeometry(
        maliput::api::RoadGeometryId("Test Dragway"), num_lanes,
        100 /* length */, kLaneWidth /* lane_width */, 0. /* shoulder_width */,
        5. /* maximum_height */,
        std::numeric_limits<double>::epsilon() /* linear_tolerance */,
        std::numeric_limits<double>::epsilon() /* angular_tolerance */));
    segment_ = road_->junction(0)->segment(0);
    ExtractLaneDirectionsFromDragway();
    right_lane_index_ = 0;
    left_lane_index_ = num_lanes - 1;

    cache_or_search_ = this->GetParam();
    period_sec_ = 1.;
  }

  // Initializes MobilPlanner with the dragway at a given initial direction of
  // travel.
  void InitializeMobilPlanner(bool initial_with_s) {
    DRAKE_DEMAND(road_ != nullptr);
    dut_.reset(new MobilPlanner<double>(
        *road_, initial_with_s, cache_or_search_, period_sec_));
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput();

    const auto mp = dynamic_cast<const MobilPlanner<double>*>(dut_.get());
    DRAKE_DEMAND(mp != nullptr);
    ego_pose_input_index_ = mp->ego_pose_input().get_index();
    ego_acceleration_input_index_ = mp->ego_acceleration_input().get_index();
    ego_velocity_input_index_ = mp->ego_velocity_input().get_index();
    traffic_input_index_ = mp->traffic_input().get_index();
    lane_output_index_ = mp->lane_output().get_index();
  }

  void ExtractLaneDirectionsFromDragway() {
    lane_directions_.clear();
    for (int i = 0; i < segment_->num_lanes(); ++i) {
      lane_directions_.push_back({segment_->lane(i), true});
    }
  }

  int get_lane_index(const Lane* lane) {
    if (segment_->lane(0)->id() == lane->id()) {
      return 0;
    } else if (segment_->lane(1)->id() == lane->id()) {
      return 1;
    } else if (segment_->lane(2)->id() == lane->id()) {
      return 2;
    } else {
      throw std::runtime_error("The specified Lane does not belong to road_.");
    }
  }

  // Creates poses for one ego car and N traffic cars situated within each of
  // the N parallel lanes in the road.
  void SetDefaultMultiLanePoses(const LaneDirection& initial_lane_direction,
                                const std::vector<double> delta_positions) {
    const int num_lanes = delta_positions.size();
    const int lane_index = get_lane_index(initial_lane_direction.lane);
    auto ego_pose = std::make_unique<PoseVector<double>>();
    auto ego_velocity = std::make_unique<FrameVelocity<double>>();
    PoseBundle<double> traffic_poses(num_lanes + 1);  // Bundles the ego and
                                                      // traffic cars.

    // Configure the ego car pose and velocity.
    const bool with_s = initial_lane_direction.with_s;
    const double length = initial_lane_direction.lane->length();
    const Eigen::Translation3d translation_ego(
        with_s ? kEgoXPosition : length - kEgoXPosition,   /* x */
        (lane_index - 0.5 * (num_lanes - 1)) * kLaneWidth, /* y */
        0.);                                               /* z */
    ego_pose->set_translation(translation_ego);
    context_->FixInputPort(ego_pose_input_index_, std::move(ego_pose));

    Vector6<double> velocity{};
    velocity << 0., /* ωx */ 0., /* ωy */ 0., /* ωz */
        kEgoSpeed, /* vx */ 0., /* vy */ 0.;  /* vz */
    ego_velocity->set_velocity(multibody::SpatialVelocity<double>(velocity));
    context_->FixInputPort(ego_velocity_input_index_, std::move(ego_velocity));

    // Mock up a command acceleration for the ego car.
    context_->FixInputPort(
        ego_acceleration_input_index_,
        systems::BasicVector<double>::Make(kEgoAccelerationCommand));

    // Configure the traffic poses and velocities, inclusive of the ego car,
    // where all cars are traveling at the same x-velocity.
    FrameVelocity<double> all_velocity;
    all_velocity.set_velocity(multibody::SpatialVelocity<double>(velocity));
    for (int i = 0; i < num_lanes; ++i) {
      const Eigen::Translation3d translation(
          translation_ego.x() + delta_positions[i], /* x */
          (i - 0.5 * (num_lanes - 1)) * kLaneWidth, /* y */
          0.);                                      /* z */
      traffic_poses.set_pose(i, Eigen::Isometry3d(translation));
      traffic_poses.set_velocity(i, all_velocity);
    }
    traffic_poses.set_pose(num_lanes, Eigen::Isometry3d(translation_ego));
    traffic_poses.set_velocity(num_lanes, all_velocity);
    context_->FixInputPort(traffic_input_index_,
                           systems::AbstractValue::Make(traffic_poses));
  }

  std::unique_ptr<systems::System<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<maliput::api::RoadGeometry> road_;
  const maliput::api::Segment* segment_;
  std::vector<LaneDirection> lane_directions_{};

  int ego_pose_input_index_{};
  int ego_velocity_input_index_{};
  int ego_acceleration_input_index_{};
  int traffic_input_index_{};
  int lane_output_index_{};

  int right_lane_index_{};
  int left_lane_index_{};

  RoadPositionStrategy cache_or_search_;
  double period_sec_{};
};

TEST_P(MobilPlannerTest, Topology) {
  InitializeDragway(2 /* num_lanes */);
  InitializeMobilPlanner(true /* initial_with_s */);

  ASSERT_EQ(4, dut_->get_num_input_ports());
  const auto& ego_pose_input_port =
      dut_->get_input_port(ego_pose_input_index_);
  EXPECT_EQ(systems::kVectorValued, ego_pose_input_port.get_data_type());
  EXPECT_EQ(7 /* PoseVector input */, ego_pose_input_port.size());
  const auto& ego_velocity_input_port =
      dut_->get_input_port(ego_velocity_input_index_);
  EXPECT_EQ(systems::kVectorValued,
            ego_velocity_input_port.get_data_type());
  EXPECT_EQ(6 /* FrameVelocity input */, ego_velocity_input_port.size());
  const auto& ego_acceleration_input_port =
      dut_->get_input_port(ego_acceleration_input_index_);
  EXPECT_EQ(systems::kVectorValued,
            ego_acceleration_input_port.get_data_type());
  EXPECT_EQ(1 /* acceleration input */,
            ego_acceleration_input_port.size());
  const auto& traffic_input_port =
      dut_->get_input_port(traffic_input_index_);
  EXPECT_EQ(systems::kAbstractValued, traffic_input_port.get_data_type());

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const auto& lane_output_port = dut_->get_output_port(lane_output_index_);
  EXPECT_EQ(systems::kAbstractValued, lane_output_port.get_data_type());
}

// Tests that the parameters index getters access the correct parameter groups.
TEST_P(MobilPlannerTest, MutableParameterAccessors) {
  InitializeDragway(2 /* num_lanes */);
  InitializeMobilPlanner(true /* initial_with_s */);

  ASSERT_EQ(2, context_->num_numeric_parameter_groups());
  const auto mobil = dynamic_cast<const MobilPlanner<double>*>(dut_.get());

  auto& mobil_params = mobil->get_mutable_mobil_params(context_.get());
  EXPECT_EQ(MobilPlannerParametersIndices::kNumCoordinates,
            mobil_params.size());
  EXPECT_TRUE(MobilPlannerParametersIndices::GetCoordinateNames() ==
              mobil_params.GetCoordinateNames());
  mobil_params[0] = 42.;

  auto& idm_params = mobil->get_mutable_idm_params(context_.get());
  EXPECT_EQ(IdmPlannerParametersIndices::kNumCoordinates, idm_params.size());
  EXPECT_TRUE(IdmPlannerParametersIndices::GetCoordinateNames() ==
              idm_params.GetCoordinateNames());
  idm_params[0] = 42.;
}

// Tests that updates to the state are correctly made.
TEST_P(MobilPlannerTest, UnrestrictedUpdate) {
  InitializeDragway(1 /* num_lanes */);
  InitializeMobilPlanner(true /* initial_with_s */);

  if (cache_or_search_ == RoadPositionStrategy::kCache) {
    EXPECT_EQ(1, context_->get_num_abstract_states());

    // Arrange the ego car in the right lane with a traffic car placed
    // arbitrarily.
    SetDefaultMultiLanePoses(lane_directions_[0], {5.});

    // Check that the unrestricted event has been registered.
    systems::LeafCompositeEventCollection<double> events;
    double t = dut_->CalcNextUpdateTime(*context_, &events);
    EXPECT_EQ(t, period_sec_);
    const systems::EventCollection<systems::UnrestrictedUpdateEvent<double>>&
        e = events.get_unrestricted_update_events();
    const auto& leaf_events = static_cast<const systems::LeafEventCollection<
      systems::UnrestrictedUpdateEvent<double>>&>(e);
    EXPECT_EQ(leaf_events.get_events().size(), 1);

    systems::State<double>& state = context_->get_mutable_state();
    dut_->CalcUnrestrictedUpdate(*context_, &state);
    const RoadPosition& rp = state.get_abstract_state<RoadPosition>(0);
    EXPECT_EQ(lane_directions_[0].lane->id(), rp.lane->id());
    EXPECT_TRUE(CompareMatrices(LanePosition{kEgoXPosition, 0., 0.}.srh(),
                                rp.pos.srh()));
  }
}

// Tests the incentive of the ego car to change lanes when tailgating a car
// close ahead.
TEST_P(MobilPlannerTest, IncentiveWhileTailgating) {
  InitializeDragway(2 /* num_lanes */);
  InitializeMobilPlanner(true /* initial_with_s */);

  // Arrange the ego car in the right lane with two traffic cars ahead, the
  // right car being closer ahead.
  SetDefaultMultiLanePoses(lane_directions_[right_lane_index_], /* ego lane */
                           {5.,    /* right car position */
                            40.}); /* left car position */

  // Compute the output.
  const auto result = output_->GetMutableData(lane_output_index_);
  dut_->CalcOutput(*context_, output_.get());
  auto lane_direction = result->template GetMutableValue<LaneDirection>();

  // Expect the left lane to be more desirable.
  EXPECT_EQ(lane_directions_[left_lane_index_].lane->id(),
            lane_direction.lane->id());
}

// Verifies that the same results as above are obtained when `with_s = false`.
TEST_P(MobilPlannerTest, IncentiveWhileTailgatingInBackwardLane) {
  // Arrange the ego car facing in the direction opposite to the canonical lane
  // direction.
  InitializeDragway(2 /* num_lanes */);
  InitializeMobilPlanner(false /* initial_with_s */);

  // Arrange two traffic cars ahead of the ego, with the right car closer ahead.
  auto reversed_right_lane = lane_directions_[right_lane_index_];
  reversed_right_lane.with_s = false;
  SetDefaultMultiLanePoses(reversed_right_lane, /* ego lane */
                           {-5.,                /* right car position */
                            -40.});             /* left car position */

  // Compute the output.
  const auto result = output_->GetMutableData(lane_output_index_);
  dut_->CalcOutput(*context_, output_.get());
  auto lane_direction = result->template GetMutableValue<LaneDirection>();

  // Expect the left lane to be more desirable.
  EXPECT_EQ(lane_directions_[left_lane_index_].lane->id(),
            lane_direction.lane->id());
}

// Tests that valid results can be obtained with no cars in the road, and that
// no exceptions are thrown.
TEST_P(MobilPlannerTest, NoCars) {
  InitializeDragway(2 /* num_lanes */);
  InitializeMobilPlanner(true /* initial_with_s */);

  // Situate the ego car, by iteself, in the right lane.
  auto right_lane = lane_directions_[right_lane_index_];
  right_lane.with_s = true;
  SetDefaultMultiLanePoses(right_lane, /* ego lane */ {}); /* car position */

  const auto result = output_->GetMutableData(lane_output_index_);
  dut_->CalcOutput(*context_, output_.get());
  auto lane_direction = result->template GetMutableValue<LaneDirection>();

  // Expect the right lane to be more desirable (the left is above threshold).
  EXPECT_EQ(lane_directions_[right_lane_index_].lane->id(),
            lane_direction.lane->id());
}

// Tests the incentive of the ego car to keep its current lane when a car is far
// ahead.
TEST_P(MobilPlannerTest, IncentiveWhileNotTailgating) {
  InitializeDragway(2 /* num_lanes */);
  InitializeMobilPlanner(true /* initial_with_s */);

  // Arrange the ego car in the left lane with two traffic cars ahead, the right
  // car being closer ahead.
  SetDefaultMultiLanePoses(lane_directions_[left_lane_index_], /* ego lane */
                           {5.,    /* right car position */
                            40.}); /* left car position */

  // Compute the output.
  const auto result = output_->GetMutableData(lane_output_index_);
  dut_->CalcOutput(*context_, output_.get());
  auto lane_direction = result->template GetMutableValue<LaneDirection>();

  // Expect the left lane to be more desirable.
  EXPECT_EQ(lane_directions_[left_lane_index_].lane->id(),
            lane_direction.lane->id());
}

// Tests the politeness of the ego car to keep its current lane when a car is
// far behind.
TEST_P(MobilPlannerTest, PolitenessNoTailgator) {
  InitializeDragway(2 /* num_lanes */);
  InitializeMobilPlanner(true /* initial_with_s */);

  // Arrange the ego car in the right lane with two traffic cars behind, the
  // left car being closer behind.
  SetDefaultMultiLanePoses(lane_directions_[right_lane_index_], /* ego lane */
                           {-40.,  /* right car position */
                            -5.}); /* left car position */

  // Compute the output.
  const auto result = output_->GetMutableData(lane_output_index_);
  dut_->CalcOutput(*context_, output_.get());
  auto lane_direction = result->template GetMutableValue<LaneDirection>();

  // Expect the right lane to be more desirable.
  EXPECT_EQ(lane_directions_[right_lane_index_].lane->id(),
            lane_direction.lane->id());
}

// Tests the politeness of the ego car to change lanes in response to being
// tailgated by a car close behind.
TEST_P(MobilPlannerTest, PolitenessWithTailgator) {
  InitializeDragway(2 /* num_lanes */);
  InitializeMobilPlanner(true /* initial_with_s */);

  // Arrange the ego car in the right lane with two traffic cars behind, the
  // right car being closer behind.
  SetDefaultMultiLanePoses(lane_directions_[right_lane_index_], /* ego lane */
                           {-5.,    /* right car position */
                            -40.}); /* left car position */

  // Compute the output.
  const auto result = output_->GetMutableData(lane_output_index_);
  dut_->CalcOutput(*context_, output_.get());
  auto lane_direction = result->template GetMutableValue<LaneDirection>();

  // Expect the left lane to be more desirable.
  EXPECT_EQ(lane_directions_[left_lane_index_].lane->id(),
            lane_direction.lane->id());
}

TEST_P(MobilPlannerTest, ThreeLanePolitenessTestPreferLeft) {
  InitializeDragway(3 /* num_lanes */);
  const int center_lane_index = 1;
  InitializeMobilPlanner(true /* initial_with_s */);

  // Arrange the ego in the middle lane with three trailing traffic cars; the
  // one behind being the closest and the leftmost car furthest behind.
  SetDefaultMultiLanePoses(lane_directions_[center_lane_index], /* ego lane */
                           {-6.,    /* rightmost car position */
                            -5,     /* car directly behind */
                            -40.}); /* leftmost car position */

  // Compute the output.
  const auto result = output_->GetMutableData(lane_output_index_);
  dut_->CalcOutput(*context_, output_.get());
  auto lane_direction = result->template GetMutableValue<LaneDirection>();

  // Expect the leftmost lane to be more desirable.
  EXPECT_EQ(lane_directions_[left_lane_index_].lane->id(),
            lane_direction.lane->id());
}

TEST_P(MobilPlannerTest, ThreeLaneIncentiveTestPreferRight) {
  InitializeDragway(3 /* num_lanes */);
  const int center_lane_index = 1;
  InitializeMobilPlanner(true /* initial_with_s */);

  // Arrange the ego in the middle lane with three trailing traffic cars; the
  // one behind being the closest and the leftmost car furthest behind.
  SetDefaultMultiLanePoses(lane_directions_[center_lane_index], /* ego lane */
                           {40.,  /* rightmost car position */
                            5,    /* car directly behind */
                            6.}); /* leftmost car position */

  // Compute the output.
  const auto result = output_->GetMutableData(lane_output_index_);
  dut_->CalcOutput(*context_, output_.get());
  auto lane_direction = result->template GetMutableValue<LaneDirection>();

  // Expect the rightmost lane to be more desirable.
  EXPECT_EQ(lane_directions_[right_lane_index_].lane->id(),
            lane_direction.lane->id());
}

// Tests that MobilPlanner is failsafe to two cars side-by-side.
TEST_P(MobilPlannerTest, SideBySideCars) {
  InitializeDragway(2 /* num_lanes */);
  InitializeMobilPlanner(true /* initial_with_s */);

  // Arrange the ego car and traffic cars to have identical poses.
  SetDefaultMultiLanePoses(lane_directions_[right_lane_index_], /* ego lane */
                           {10., 0.}); /* traffic car position */

  // Expect failsafe behavior.
  const auto result = output_->GetMutableData(lane_output_index_);
  dut_->CalcOutput(*context_, output_.get());
  auto lane_direction = result->template GetMutableValue<LaneDirection>();

  // Expect the rightmost lane (ego car's lane) to be more desirable.
  EXPECT_EQ(lane_directions_[right_lane_index_].lane->id(),
            lane_direction.lane->id());
}

// Perform all tests with cache and exhaustive search options.
INSTANTIATE_TEST_CASE_P(
    RoadPositionStrategy, MobilPlannerTest,
    testing::Values(RoadPositionStrategy::kCache,
                    RoadPositionStrategy::kExhaustiveSearch));

}  // namespace
}  // namespace automotive
}  // namespace drake
