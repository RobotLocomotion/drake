#include "drake/automotive/idm_controller.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"

namespace drake {
namespace automotive {
namespace {

using maliput::dragway::RoadGeometry;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;
using systems::rendering::PoseBundle;

constexpr double kEgoSPosition{10.};

constexpr int kLeadIndex{0};
constexpr int kEgoIndex{1};

class IdmControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a straight road with one lane.
    road_.reset(new maliput::dragway::RoadGeometry(
        maliput::api::RoadGeometryId({"Single-Lane Dragway"}),
        1 /* num_lanes */, 100. /* length */, 2. /* lane_width */,
        0. /* shoulder_width */));

    // Initialize IdmController with the road.
    dut_.reset(new IdmController<double>(std::move(road_)));
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
  }

  systems::ContinuousState<double>* continuous_state() {
    return context_->get_mutable_continuous_state();
  }

  std::unique_ptr<IdmController<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<maliput::dragway::RoadGeometry> road_;

  // Set the default poses according to the desired offset position and speeds
  // for the lead car.  `s_offset = s_lead - s_ego` and `speed_offset =
  // speed_lead - speed_ego`.
  void SetDefaultPoses(const double ego_speed, const double s_offset,
                       const double relative_speed) {
    DRAKE_DEMAND(s_offset > 0.);

    // Configure the ego car pose and velocity.
    auto ego_pose = std::make_unique<PoseVector<double>>();
    const Eigen::Translation3d translation_ego(kEgoSPosition /* x */,
                                               0. /* y */, 0. /* z */);
    ego_pose->set_translation(translation_ego);
    context_->FixInputPort(dut_->ego_pose_input().get_index(),
                           std::move(ego_pose));

    auto ego_velocity = std::make_unique<FrameVelocity<double>>();
    Vector6<double> velocity{};
    velocity << 0. /* p */, 0. /* q */, 0. /* r */, ego_speed /* u */,
        0. /* v */, 0. /* w */;
    ego_velocity->set_velocity(multibody::SpatialVelocity<double>(velocity));
    context_->FixInputPort(dut_->ego_velocity_input().get_index(),
                           std::move(ego_velocity));

    // Configure the traffic poses, inclusive of the ego car.
    PoseBundle<double> traffic_poses(2);
    FrameVelocity<double> lead_velocity;
    const Eigen::Translation3d translation_lead(
        kEgoSPosition + s_offset /* x */, 0. /* y */, 0. /* z */);
    traffic_poses.set_pose(kLeadIndex, Eigen::Isometry3d(translation_lead));
    velocity[3] += relative_speed;
    lead_velocity.set_velocity(multibody::SpatialVelocity<double>(velocity));
    traffic_poses.set_velocity(kLeadIndex, lead_velocity);
    traffic_poses.set_pose(kEgoIndex, Eigen::Isometry3d(translation_ego));
    context_->FixInputPort(dut_->traffic_input().get_index(),
                           systems::AbstractValue::Make(traffic_poses));
  }
};

TEST_F(IdmControllerTest, Topology) {
  ASSERT_EQ(3, dut_->get_num_input_ports());
  const auto& ego_pose_input_descriptor = dut_->get_input_port(0);
  EXPECT_EQ(systems::kVectorValued, ego_pose_input_descriptor.get_data_type());
  EXPECT_EQ(7 /* PoseVector input */, ego_pose_input_descriptor.size());
  const auto& ego_velocity_input_descriptor = dut_->get_input_port(1);
  EXPECT_EQ(systems::kVectorValued,
            ego_velocity_input_descriptor.get_data_type());
  EXPECT_EQ(6 /* FrameVelocity input */, ego_velocity_input_descriptor.size());
  const auto& traffic_input_descriptor = dut_->get_input_port(2);
  EXPECT_EQ(systems::kAbstractValued, traffic_input_descriptor.get_data_type());

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const auto& output_descriptor = dut_->get_output_port(0);
  EXPECT_EQ(systems::kVectorValued, output_descriptor.get_data_type());
  EXPECT_EQ(3 /* DrivingCommand output */, output_descriptor.size());
}

TEST_F(IdmControllerTest, Output) {
  // Define a pointer to where the DrivingCommand results end up.
  const auto result =
      output_->get_vector_data(dut_->get_output_port(0).get_index());
  const auto command = dynamic_cast<const DrivingCommand<double>*>(result);

  // Set the lead car to be immediately ahead of the ego car and slower.
  SetDefaultPoses(10. /* ego_speed */, 3. /* s_offset */, -5. /* rel_speed */);
  dut_->CalcOutput(*context_, output_.get());
  const double closing_brake = command->brake();

  // Expect the car to brake in this configuration, and therefore zero throttle.
  EXPECT_EQ(0., command->throttle());
  EXPECT_LT(0., closing_brake);
  EXPECT_EQ(0., command->steering_angle());  // Always zero.

  // Set the lead car to be immediately ahead of the ego car and faster.
  SetDefaultPoses(10. /* ego_speed */, 3. /* s_offset */, 5. /* rel_speed */);
  dut_->CalcOutput(*context_, output_.get());

  // Expect the magnitude of the braking to be smaller when the ego is not
  // closing in on the lead car.
  EXPECT_LT(command->brake(), closing_brake);

  // Set the leading car to be far away from the ego car.
  SetDefaultPoses(10. /* ego_speed */, 1e12 /* s_offset */, 0. /* rel_speed */);
  dut_->CalcOutput(*context_, output_.get());

  // Expect no braking or throttling (input velocity is at the desired
  // velocity).
  EXPECT_EQ(0., command->throttle());
  EXPECT_NEAR(0., command->brake(), 1e-4);

  // Set the lead car well ahead of the ego, with the ego moving slower than the
  // desired velocity.
  SetDefaultPoses(4. /* ego_speed */, 30. /* s_offset */, 0. /* rel_speed */);
  dut_->CalcOutput(*context_, output_.get());

  // Expect a throttle input in this confguration (no brake).
  EXPECT_LT(0., command->throttle());
  EXPECT_EQ(0., command->brake());
}

}  // namespace
}  // namespace automotive
}  // namespace drake
