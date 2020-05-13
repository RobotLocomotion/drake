#include "drake/manipulation/perception/optitrack_pose_extractor.h"

#include <memory>

#include "optitrack/optitrack_frame_t.hpp"
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace manipulation {
namespace perception {

// @note This tolerance is relatively loose (most likely) because the Optitrack
// LCM message types use single- rather than double-precision floating point
// types for quaternions.
constexpr double kTolerance = 1e-6;

class OptitrackPoseTest : public ::testing::Test {
 public:
  void Initialize(int object_id = 0,
                  const math::RigidTransform<double>& world_X_optitrack =
                        math::RigidTransform<double>::Identity()) {
    dut_ = std::make_unique<OptitrackPoseExtractor>(
        object_id, world_X_optitrack.GetAsIsometry3(),
        0.01 /* optitrack_lcm_status_period */);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput();

    EXPECT_EQ(dut_->num_input_ports(), 1);
    EXPECT_EQ(dut_->num_output_ports(), 1);
  }

  math::RigidTransform<double> UpdateStateCalcOutput(
      const optitrack::optitrack_frame_t& input_frame) {
    dut_->get_input_port(0).FixValue(context_.get(), input_frame);

    dut_->CalcUnrestrictedUpdate(*context_, &context_->get_mutable_state());
    dut_->CalcOutput(*context_, output_.get());
    auto output_value = output_->get_data(0);

    const Isometry3<double> iso = output_value->get_value<Isometry3<double>>();
    return math::RigidTransform<double>(iso);
  }

 private:
  std::unique_ptr<OptitrackPoseExtractor> dut_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
};

TEST_F(OptitrackPoseTest, InvalidObjectTest) {
  const int object_id = 2;
  Initialize(object_id);
  optitrack::optitrack_frame_t test_frame{};
  optitrack::optitrack_rigid_body_t default_body{};
  default_body.id = 0;

  // Need to initialize default_body.quat to a valid quaternion or else there
  // is a downstream RotationMatrix class that throws an exception because
  // the quaternion is invalid (e.g., it has NANs).
  // Exceptions being thrown by the RotationMatrix class are not what this test
  // is trying to achieve -- so properly initialize default_body.quat.
  default_body.quat[0] = 1;
  default_body.quat[1] = 0;
  default_body.quat[2] = 0;
  default_body.quat[3] = 0;

  test_frame.rigid_bodies.push_back(default_body);
  default_body.id = 1;
  test_frame.rigid_bodies.push_back(default_body);
  // Test frame has only 2 bodies but DUT (Device Under Test) extracts pose of
  // non-existent 3rd object (object ID = 2)
  EXPECT_ANY_THROW(UpdateStateCalcOutput(test_frame));

  // Adding the appropriate number of bodies to the test frame will result
  // in an update with no errors thrown.
  default_body.id = 2;
  test_frame.rigid_bodies.push_back(default_body);
  DRAKE_EXPECT_NO_THROW(UpdateStateCalcOutput(test_frame));
}

TEST_F(OptitrackPoseTest, InvalidObjectIDTest) {
  const int object_id = 1;
  Initialize(object_id);
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
  const int object_id = 0;
  const math::RigidTransform<double> X_WO(
      Eigen::AngleAxisd(-0.75 * M_PI, Eigen::Vector3d::UnitX()),
      Vector3<double>(1, 2, 3) );
  Initialize(object_id, X_WO);
  optitrack::optitrack_frame_t test_frame{};
  optitrack::optitrack_rigid_body_t default_body{};

  // An arbitrarily chosen test pose is assigned to the default_body.
  const math::RigidTransform<double> X_OB_expected(
      Eigen::AngleAxisd(0.75 * M_PI, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(-0.75 * M_PI, Eigen::Vector3d::UnitY()),
      Vector3<double>(0.01, -5.0, 10.10));

  default_body.xyz[0] = X_OB_expected.translation()[0];
  default_body.xyz[1] = X_OB_expected.translation()[1];
  default_body.xyz[2] = X_OB_expected.translation()[2];

  const Eigen::Quaterniond test_pose_quaternion =
      X_OB_expected.rotation().ToQuaternion();
  default_body.quat[0] = test_pose_quaternion.x();
  default_body.quat[1] = test_pose_quaternion.y();
  default_body.quat[2] = test_pose_quaternion.z();
  default_body.quat[3] = test_pose_quaternion.w();
  default_body.id = 0;

  test_frame.rigid_bodies.push_back(default_body);

  // Non-systems tests.

  // - `ExtractPose`.
  const Isometry3<double> X_OB_direct =
      ExtractOptitrackPose(*FindOptitrackBody(test_frame, object_id));
  EXPECT_TRUE(CompareMatrices(
      X_OB_direct.matrix(), X_OB_expected.GetAsMatrix4(),
      kTolerance, MatrixCompareType::absolute));

  // - `ExtractOptitrackPoses`.
  using Map = std::map<int, Isometry3<double>>;
  const Map all_poses = ExtractOptitrackPoses(test_frame);
  const Map all_poses_expected = {{0, X_OB_expected.GetAsIsometry3()}};
  EXPECT_EQ(all_poses.size(), all_poses_expected.size());
  for (auto& pair : all_poses) {
    Isometry3<double> X_OBi = pair.second;
    Isometry3<double> X_OBi_expected = all_poses_expected.at(pair.first);
    EXPECT_TRUE(CompareMatrices(
        X_OBi.matrix(), X_OBi_expected.matrix(),
        kTolerance, MatrixCompareType::absolute));
  }

  // - Negative test for `FindOptitrackBody`.
  EXPECT_FALSE(FindOptitrackBody(test_frame, 999).has_value());

  // Systems test.
  const math::RigidTransform<double> X_WB_expected = X_WO * X_OB_expected;
  math::RigidTransform<double> X_WB;
  DRAKE_EXPECT_NO_THROW(X_WB = UpdateStateCalcOutput(test_frame));

  // Compare.
  EXPECT_TRUE(X_WB.IsNearlyEqualTo(X_WB_expected, kTolerance));
}

TEST_F(OptitrackPoseTest, FindObject) {
  optitrack::optitrack_data_descriptions_t message;
  message.rigid_bodies = {{"test", 10}};
  message.num_rigid_bodies = message.rigid_bodies.size();

  EXPECT_EQ(*FindOptitrackObjectId(message, "test"), 10);
  EXPECT_FALSE(FindOptitrackObjectId(message, "blergh").has_value());
}

}  // namespace perception
}  // namespace manipulation
}  // namespace drake
