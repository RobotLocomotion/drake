#include "drake/systems/sensors/optitrack_sender.h"

#include <gtest/gtest.h>
#include "optitrack/optitrack_frame_t.hpp"

#include "drake/systems/framework/context.h"
#include "drake/systems/sensors/optitrack_encoder.h"

namespace drake {
namespace systems {
namespace sensors {

using systems::sensors::TrackedBody;
using optitrack::optitrack_frame_t;

// This test sets up a systems::sensors::TrackedBody structure to be passed
// through by the OptitrackLCMFrameSender test object. The output should be
// an abstract value object templated on type `optitrack_frame_t`. This test
// ensures that the TrackedBody input to the OptitrackLCMFrameSender system
// matches the output `optitrack_frame_t` object.
GTEST_TEST(OptitrackSenderTest, OptitrackLCMSenderTest) {
  OptitrackLCMFrameSender dut(1);  // Create a frame sender with 1 rigid body.
  std::vector<TrackedBody> tracked_body(1);

  constexpr double tx = 0.2;  // x-translation for the test object
  constexpr double ty = 0.4;  // y-translation for the test object
  constexpr double tz = 0.7;  // z-translation for the test object

  // Sets up a test body with an arbitrarily chosen pose.
  Eigen::Vector3d axis(1 / sqrt(3), 1 / sqrt(3), 1 / sqrt(3));
  tracked_body[0].id = 1;
  tracked_body[0].body_name = "test_body";
  tracked_body[0].T_WF = Eigen::Isometry3d(Eigen::AngleAxis<double>(0.2, axis)).
      pretranslate(Eigen::Vector3d(tx, ty, tz));

  EXPECT_EQ(tracked_body[0].T_WF.translation()[0], tx);
  EXPECT_EQ(tracked_body[0].T_WF.translation()[1], ty);
  EXPECT_EQ(tracked_body[0].T_WF.translation()[2], tz);

  std::unique_ptr<systems::AbstractValue> input(
      new systems::Value<std::vector<TrackedBody>>());
  input->SetValue(tracked_body);

  auto context = dut.CreateDefaultContext();
  auto output = dut.AllocateOutput(*context);
  context->FixInputPort(0 /* input port ID*/, std::move(input));

  dut.CalcUnrestrictedUpdate(*context, &context->get_mutable_state());
  dut.CalcOutput(*context, output.get());
  auto output_value = output->get_data(0);

  auto lcm_frame =  output_value->GetValue<optitrack_frame_t>();

  // Compare the resultant lcm_frame values to those provided at the input to
  // the system.
  EXPECT_EQ(lcm_frame.utime, 0);
  EXPECT_EQ(lcm_frame.num_rigid_bodies, 1);

  // Check the translations. Allow for some numerical error.
  EXPECT_NEAR(lcm_frame.rigid_bodies[0].xyz[0], tx, 1e-7);
  EXPECT_NEAR(lcm_frame.rigid_bodies[0].xyz[1], ty, 1e-7);
  EXPECT_NEAR(lcm_frame.rigid_bodies[0].xyz[2], tz, 1e-7);

  // Check the rotations.
  const Eigen::Quaterniond quat_rotation = Eigen::Quaterniond(
      lcm_frame.rigid_bodies[0].quat[3], lcm_frame.rigid_bodies[0].quat[0],
      lcm_frame.rigid_bodies[0].quat[1], lcm_frame.rigid_bodies[0].quat[2]);
  Eigen::Isometry3d body_pose = Eigen::Isometry3d(quat_rotation).
      pretranslate(Eigen::Vector3d(tx, ty, tz));
  EXPECT_TRUE(body_pose.isApprox(tracked_body[0].T_WF, 1e-1));
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
