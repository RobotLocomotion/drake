#include "drake/systems/sensors/optitrack_receiver.h"

#include "optitrack/optitrack_frame_t.hpp"
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {

using math::RigidTransformd;

// Checks that body poses are passed through correctly.
GTEST_TEST(OptitrackPoseTest, InputOutputTest) {
  // Set up the device under test, with two tracked bodies.
  const std::map<int, std::string> frame_map{{1, "one"}, {2, "two"}};
  const RigidTransformd X_WO(
      Eigen::AngleAxisd(-0.75 * M_PI, Eigen::Vector3d::UnitX()),
      Vector3<double>(1, 2, 3));
  const OptitrackReceiver dut(frame_map, X_WO);
  EXPECT_EQ(dut.num_input_ports(), 1);
  EXPECT_EQ(dut.num_output_ports(), 2);

  // Use an arbitrarily chosen pose for body 1.
  const math::RigidTransform<double> X_OB1(
      Eigen::AngleAxisd(0.75 * M_PI, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(-0.75 * M_PI, Eigen::Vector3d::UnitY()),
      Vector3<double>(0.01, -5.0, 10.10));
  const Eigen::Quaterniond R_OB1_quat = X_OB1.rotation().ToQuaternion();

  // Encode body 1 into LCM.
  optitrack::optitrack_rigid_body_t body_1{};
  body_1.id = 1;
  body_1.xyz[0] = X_OB1.translation()[0];
  body_1.xyz[1] = X_OB1.translation()[1];
  body_1.xyz[2] = X_OB1.translation()[2];
  body_1.quat[0] = R_OB1_quat.x();
  body_1.quat[1] = R_OB1_quat.y();
  body_1.quat[2] = R_OB1_quat.z();
  body_1.quat[3] = R_OB1_quat.w();

  // Encode body 2 (posed at the origin) into LCM.
  optitrack::optitrack_rigid_body_t body_2{};
  body_2.id = 2;
  body_2.quat[3] = 1.0 /* w */;

  // Copy the message into the context.
  optitrack::optitrack_frame_t frame{};
  frame.rigid_bodies.push_back(body_2);
  frame.rigid_bodies.push_back(body_1);
  auto context = dut.CreateDefaultContext();
  dut.get_input_port().FixValue(context.get(), frame);

  // Check each output. This tolerance is relatively loose because the LCM
  // message types use single-precision floating point types for quaternions.
  constexpr double kTolerance = 1e-6;
  const auto& out_1 = dut.GetOutputPort("one").Eval<RigidTransformd>(*context);
  const auto& out_2 = dut.GetOutputPort("two").Eval<RigidTransformd>(*context);
  EXPECT_TRUE(CompareMatrices(
      out_1.GetAsMatrix34(), (X_WO * X_OB1).GetAsMatrix34(),
      kTolerance, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      out_2.GetAsMatrix34(), X_WO.GetAsMatrix34(),
      kTolerance, MatrixCompareType::absolute));

  // If a body is missing from the message, that is an error.
  frame.rigid_bodies.clear();
  dut.get_input_port().FixValue(context.get(), frame);
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.GetOutputPort("one").Eval<RigidTransformd>(*context),
      ".*does not contain body.*");
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
