#include "drake/automotive/pure_pursuit_controller.h"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace automotive {
namespace {

using maliput::dragway::RoadGeometry;
using systems::rendering::PoseVector;

constexpr double kXPosition{10.};
constexpr double kLaneWidth{4.};

class PurePursuitControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a straight road with two lanes.
    road_.reset(new maliput::dragway::RoadGeometry(
        maliput::api::RoadGeometryId("Two-Lane Dragway"), 1 /* num_lanes */,
        100 /* length */, kLaneWidth /* lane_width */, 0. /* shoulder_width */,
        5. /* maximum_height */,
        std::numeric_limits<double>::epsilon() /* linear_tolerance */,
        std::numeric_limits<double>::epsilon() /* angular_tolerance */));

    // Store the LaneDirection.
    lane_direction_.reset(
        new LaneDirection(road_->junction(0)->segment(0)->lane(0), true));

    // Initialize PurePursuitController with the dragway.
    dut_.reset(new PurePursuitController<double>());
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
  }

  // Create poses for one ego car and two traffic cars.
  void SetDefaultInputs(const double y_position, const double yaw) {
    // Set the LaneId.
    context_->FixInputPort(dut_->lane_input().get_index(),
                           systems::AbstractValue::Make(*lane_direction_));

    // Set the ego car's pose.
    auto ego_pose = std::make_unique<PoseVector<double>>();
    const Eigen::Translation3d translation(kXPosition /* x */,
                                           y_position /* y */, 0. /* z */);
    const Eigen::Quaternion<double> rotation(std::cos(yaw * 0.5) /* w */,
                                             0. /* x */, 0. /* y */,
                                             std::sin(yaw * 0.5) /* z */);
    ego_pose->set_translation(translation);
    ego_pose->set_rotation(rotation);
    context_->FixInputPort(dut_->ego_pose_input().get_index(),
                           std::move(ego_pose));
  }

  std::unique_ptr<PurePursuitController<double>> dut_;  //< The device under
                                                        //  test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<maliput::api::RoadGeometry> road_;
  std::unique_ptr<LaneDirection> lane_direction_;
};

TEST_F(PurePursuitControllerTest, Topology) {
  ASSERT_EQ(2, dut_->get_num_input_ports());
  const auto& lane_input_descriptor =
      dut_->get_input_port(dut_->lane_input().get_index());
  EXPECT_EQ(systems::kAbstractValued, lane_input_descriptor.get_data_type());
  const auto& ego_input_descriptor =
      dut_->get_input_port(dut_->ego_pose_input().get_index());
  EXPECT_EQ(systems::kVectorValued, ego_input_descriptor.get_data_type());
  EXPECT_EQ(7 /* PoseVector input */, ego_input_descriptor.size());

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const auto& command_output_port =
      dut_->get_output_port(dut_->steering_command_output().get_index());
  EXPECT_EQ(systems::kVectorValued, command_output_port.get_data_type());
  EXPECT_EQ(1 /* steering angle output */, command_output_port.size());
}

TEST_F(PurePursuitControllerTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*dut_, [&](const auto& other_dut) {
    auto other_context = other_dut.CreateDefaultContext();
    auto other_output = other_dut.AllocateOutput(*other_context);
    auto other_derivatives = other_dut.AllocateTimeDerivatives();

    other_context->FixInputPort(dut_->lane_input().get_index(),
                                systems::AbstractValue::Make(*lane_direction_));
    auto ego_pose = std::make_unique<PoseVector<AutoDiffXd>>();
    other_context->FixInputPort(dut_->ego_pose_input().get_index(),
                                std::move(ego_pose));

    other_dut.CalcOutput(*other_context, other_output.get());
  }));
}

TEST_F(PurePursuitControllerTest, Output) {
  // Define a pointer to where the DrivingCommand results end up.
  const auto result =
      output_->get_vector_data(dut_->steering_command_output().get_index());
  ASSERT_NE(nullptr, result);

  // Set the offset to be to one the centerline with zero orientation.
  SetDefaultInputs(0., 0.);
  dut_->CalcOutput(*context_, output_.get());

  // Expect steering to be zero.
  EXPECT_EQ(0., (*result)[0]);

  // Set the offset to be to the right of the centerline with zero orientation.
  SetDefaultInputs(-1., 0.);
  dut_->CalcOutput(*context_, output_.get());

  // Expect the car to steer toward the left (positive steering angle).
  EXPECT_LT(0., (*result)[0]);

  // Set the offset to be to the left of the centerline, oriented at 90 degrees
  // with respect to the track.
  SetDefaultInputs(1., -M_PI_2);
  dut_->CalcOutput(*context_, output_.get());

  // Expect the car to steer toward the left (positive steering angle).
  EXPECT_LT(0., (*result)[0]);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
