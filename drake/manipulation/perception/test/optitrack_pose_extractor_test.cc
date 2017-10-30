#include "drake/manipulation/perception/optitrack_pose_extractor.h"

#include <memory>

#include <gtest/gtest.h>
#include "optitrack/optitrack_frame_t.hpp"

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/input_port_value.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace manipulation {
namespace perception {

class OptitrackPoseTest : public ::testing::Test {
 public:
  void Initialize(int object_id = 0,
                  const Isometry3<double>& world_X_optitrack =
                      Isometry3<double>::Identity()) {
    dut_ = std::make_unique<OptitrackPoseExtractor>(
        object_id, world_X_optitrack, 0.01 /* optitrack_lcm_status_period */);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);

    EXPECT_EQ(dut_->get_num_input_ports(), 1);
    EXPECT_EQ(dut_->get_num_output_ports(), 1);
  }

  Isometry3<double> UpdateStateCalcOutput(
      const optitrack::optitrack_frame_t& input_frame) {
    std::unique_ptr<systems::AbstractValue> input(
        new systems::Value<optitrack::optitrack_frame_t>());
    input->SetValue(input_frame);
    context_->FixInputPort(0 /* input port ID*/, std::move(input));

    dut_->CalcUnrestrictedUpdate(*context_, &context_->get_mutable_state());
    dut_->CalcOutput(*context_, output_.get());
    auto output_value = output_->get_data(0);

    return output_value->GetValue<Isometry3<double>>();
  }

 private:
  std::unique_ptr<OptitrackPoseExtractor> dut_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
};

TEST_F(OptitrackPoseTest, InvalidObjectTest) {
  Initialize(2 /* object_id */);
  optitrack::optitrack_frame_t test_frame{};
  optitrack::optitrack_rigid_body_t default_body{};
  default_body.id = 0;

  test_frame.rigid_bodies.push_back(default_body);
  default_body.id = 1;
  test_frame.rigid_bodies.push_back(default_body);
  // Test frame has only 2 bodies but DUT extracts pose of non-existent
  // 3rd object (object ID = 2)
  EXPECT_ANY_THROW(UpdateStateCalcOutput(test_frame));

  // Adding the appropriate number of bodies to the test frame will result
  // in an update with no errors thrown.
  default_body.id = 2;
  test_frame.rigid_bodies.push_back(default_body);
  EXPECT_NO_THROW(UpdateStateCalcOutput(test_frame));
}

TEST_F(OptitrackPoseTest, InvalidObjectIDTest) {
  Initialize(1 /* object_id */);
  optitrack::optitrack_frame_t test_frame{};
  optitrack::optitrack_rigid_body_t default_body{};
  default_body.id = 0;
  test_frame.rigid_bodies.push_back(default_body);
  default_body.id = 2;
  test_frame.rigid_bodies.push_back(default_body);
  // Test frame has 2 bodies but DUT extracts pose of non-existent object_id
  // of the 2nd object (object ID = 2 not 1)
  EXPECT_ANY_THROW(UpdateStateCalcOutput(test_frame));
}

TEST_F(OptitrackPoseTest, PoseComparisonTest) {
  Initialize(0 /* object_id */);
  optitrack::optitrack_frame_t test_frame{};
  optitrack::optitrack_rigid_body_t default_body{};

  // An arbitrarily chosen test pose is assigned to the default_body.
  Isometry3<double> test_pose;
  test_pose.translation() << 0.01, -5.0, 10.10;

  test_pose.linear() =
      Eigen::AngleAxisd(0.75 * M_PI, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(-0.75 * M_PI, Eigen::Vector3d::UnitY()).matrix();

  default_body.xyz[0] = test_pose.translation()[0];
  default_body.xyz[1] = test_pose.translation()[1];
  default_body.xyz[2] = test_pose.translation()[2];

  Eigen::Quaterniond test_pose_quaternion(test_pose.linear());
  default_body.quat[0] = test_pose_quaternion.x();
  default_body.quat[1] = test_pose_quaternion.y();
  default_body.quat[2] = test_pose_quaternion.z();
  default_body.quat[3] = test_pose_quaternion.w();
  default_body.id = 0;

  test_frame.rigid_bodies.push_back(default_body);

  Isometry3<double> extracted_pose;
  EXPECT_NO_THROW(extracted_pose = UpdateStateCalcOutput(test_frame));

  // Compare translation.
  EXPECT_TRUE(CompareMatrices(extracted_pose.translation(),
                              test_pose.translation(), 1e-3,
                              MatrixCompareType::absolute));

  // Compare quaternions.
  EXPECT_TRUE(CompareMatrices(extracted_pose.linear(), test_pose.linear(),
                              1e-3, MatrixCompareType::absolute));
}

TEST_F(OptitrackPoseTest, PoseInReferenceFrameTest) {
  // Arbitrarily chosen Transform from Drake World frame to Optitrack Frame.
  Isometry3<double> X_WOp;
  X_WOp.translation() << 7, 10, -4.5;
  X_WOp.linear() =
      Eigen::AngleAxisd(-0.75 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  Initialize(0 /* object_id */, X_WOp);

  optitrack::optitrack_frame_t test_frame{};
  optitrack::optitrack_rigid_body_t default_body{};

  // An arbitrarily chosen test pose is assigned to the default_body.
  Isometry3<double> test_pose;
  test_pose.translation() << 0.01, -5.0, 10.10;

  test_pose.linear() =
      Eigen::AngleAxisd(0.75 * M_PI, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(-0.75 * M_PI, Eigen::Vector3d::UnitY()).matrix();

  default_body.xyz[0] = test_pose.translation()[0];
  default_body.xyz[1] = test_pose.translation()[1];
  default_body.xyz[2] = test_pose.translation()[2];

  Eigen::Quaterniond test_pose_quaternion(test_pose.linear());
  default_body.quat[0] = test_pose_quaternion.x();
  default_body.quat[1] = test_pose_quaternion.y();
  default_body.quat[2] = test_pose_quaternion.z();
  default_body.quat[3] = test_pose_quaternion.w();
  default_body.id = 0;

  test_frame.rigid_bodies.push_back(default_body);

  Isometry3<double> transformed_test_pose = X_WOp * test_pose;

  Isometry3<double> extracted_pose;
  EXPECT_NO_THROW(extracted_pose = UpdateStateCalcOutput(test_frame));

  EXPECT_TRUE(CompareMatrices(extracted_pose.translation(),
                              transformed_test_pose.translation(), 1e-3,
                              MatrixCompareType::absolute));

  // Compare quaternions.
  EXPECT_TRUE(CompareMatrices(extracted_pose.linear(),
                              transformed_test_pose.linear(), 1e-3,
                              MatrixCompareType::absolute));
}

}  // namespace perception
}  // namespace manipulation
}  // namespace drake
