#pragma once

#include <memory>
#include <utility>

#include "gtest/gtest.h"

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace automotive {

using systems::rendering::FrameVelocity;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

constexpr double kEgoSPosition{10.};
constexpr int kLeadIndex{0};
constexpr int kEgoIndex{1};

// Base class for IdmControllerTest and MobilPlannerTest.
class IdmTest : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IdmTest)

  IdmTest() = default;

  void SetUp() override { Initialize(); }

  virtual void Initialize() = 0;

  // Set the default poses according to the desired offset position and speeds
  // for the lead car.  `s_offset = s_lead - s_ego`, `relative_sdot =
  // sdot_lead - sdot_ego`, and `relative_rdot = rdot_lead - rdot_ego`.
  void SetDefaultPoses(const double ego_speed, const double s_offset = 0.,
                       const double relative_sdot = 0.,
                       const double relative_rdot = 0.) {
    DRAKE_DEMAND(ego_pose_input_index_ >= 0 &&
                 ego_pose_input_index_ < dut_->get_num_input_ports());
    DRAKE_DEMAND(ego_velocity_input_index_ >= 0 &&
                 ego_velocity_input_index_ < dut_->get_num_input_ports());
    DRAKE_DEMAND(traffic_input_index_ >= 0 &&
                 traffic_input_index_ < dut_->get_num_input_ports());

    auto ego_pose = std::make_unique<PoseVector<double>>();
    auto ego_velocity = std::make_unique<FrameVelocity<double>>();
    PoseBundle<double> traffic_poses(2);

    DRAKE_DEMAND(s_offset >= 0.);
    EXPECT_LE(0., ego_speed);

    // Configure the ego car pose and velocity.
    const Eigen::Translation3d translation_ego(kEgoSPosition /* x */,
                                               0. /* y */, 0. /* z */);
    ego_pose->set_translation(translation_ego);
    context_->FixInputPort(ego_pose_input_index_, std::move(ego_pose));

    Vector6<double> velocity{};
    velocity << 0. /* ωx */, 0. /* ωy */, 0. /* ωz */, ego_speed /* vx */,
        0. /* vy */, 0. /* vz */;
    ego_velocity->set_velocity(multibody::SpatialVelocity<double>(velocity));
    context_->FixInputPort(ego_velocity_input_index_, std::move(ego_velocity));

    // Configure the traffic poses, inclusive of the ego car.
    FrameVelocity<double> lead_velocity;
    const Eigen::Translation3d translation_lead(
        kEgoSPosition + s_offset /* x */, 0. /* y */, 0. /* z */);
    traffic_poses.set_pose(kLeadIndex, Eigen::Isometry3d(translation_lead));
    velocity[3] += relative_sdot;
    velocity[4] += relative_rdot;
    lead_velocity.set_velocity(multibody::SpatialVelocity<double>(velocity));
    traffic_poses.set_velocity(kLeadIndex, lead_velocity);
    traffic_poses.set_pose(kEgoIndex, Eigen::Isometry3d(translation_ego));
    context_->FixInputPort(traffic_input_index_,
                           systems::AbstractValue::Make(traffic_poses));
  }

  void TestIdmPlanner(const DrivingCommand<double>& command) {
    // Set the lead car to be immediately ahead of the ego car and moving
    // slower.
    SetDefaultPoses(10. /* ego_speed */, 6. /* s_offset */, -5. /* rel_sdot */);
    dut_->CalcOutput(*context_, output_.get());
    const double closing_accel = command.acceleration();  // A negative number.

    // Expect the car to decelerate in this configuration.
    EXPECT_GT(0., closing_accel);
    EXPECT_EQ(0., command.steering_angle());  // Always zero.

    // Set the same conditions as above, but with the lead car having a nonzero
    // r-component in its velocity.
    SetDefaultPoses(10. /* ego_speed */, 6. /* s_offset */, -5. /* rel_sdot */,
                    5. /* rel_rdot */);
    dut_->CalcOutput(*context_, output_.get());

    // Expect no change to the previous results.
    EXPECT_EQ(closing_accel, command.acceleration());

    // Set the lead car to be immediately ahead of the ego car and moving
    // faster.
    SetDefaultPoses(10. /* ego_speed */, 6. /* s_offset */, 5. /* rel_sdot */);
    dut_->CalcOutput(*context_, output_.get());

    // Expect the magnitude of the deceleration to be smaller when the ego is
    // not closing in on the lead car.
    EXPECT_GT(command.acceleration(), closing_accel);

    // Set the ego car to be alone on the road.  We effectively enable this by
    // setting the poses of all traffic cars to be that of the ego car.
    SetDefaultPoses(10. /* ego_speed */);
    dut_->CalcOutput(*context_, output_.get());

    // Expect zero acceleration (input velocity is at the desired velocity).
    EXPECT_NEAR(0., command.acceleration(), 1e-4);

    // Set the lead car well ahead of the ego, with the ego moving slower than
    // the desired velocity.
    SetDefaultPoses(4. /* ego_speed */, 30. /* s_offset */, 0. /* rel_sdot */);
    dut_->CalcOutput(*context_, output_.get());

    // Expect a positive acceleration in this configuration.
    EXPECT_LT(0., command.acceleration());

    // Set the lead car to be well within `distance_lower_limit`.
    SetDefaultPoses(10. /* ego_speed */, 1e-3 /* s_offset */,
                    -5. /* rel_sdot */);
    dut_->CalcOutput(*context_, output_.get());

    // Expect an enormous deceleration.
    EXPECT_GT(closing_accel, command.acceleration());
  }

 protected:
  std::unique_ptr<systems::System<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;

  int ego_pose_input_index_;
  int ego_velocity_input_index_;
  int traffic_input_index_;
};

}  // namespace automotive
}  // namespace drake
