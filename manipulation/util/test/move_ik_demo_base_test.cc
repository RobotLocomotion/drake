#include "drake/manipulation/util/move_ik_demo_base.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"

namespace drake {
namespace manipulation {
namespace util {

const char kIiwaUrdf[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_no_collision.urdf";

GTEST_TEST(MoveIkDemoBaseTest, IiwaTest) {
  math::RigidTransformd pose(
      math::RollPitchYawd(0, 0, -1.57),
      Eigen::Vector3d(0.8, -0.3, 0.25));

  MoveIkDemoBase dut(
      FindResourceOrThrow(kIiwaUrdf),
      "base", "iiwa_link_ee", 100);
  dut.set_joint_velocity_limits(kuka_iiwa::get_iiwa_max_joint_velocities());
  dut.HandleStatus(Eigen::VectorXd::Ones(7));
  auto plan = dut.Plan(pose);
  EXPECT_TRUE(plan.has_value());

  dut.HandleStatus(Eigen::VectorXd::Ones(7) * 1.1);
  pose.set_translation(Eigen::Vector3d(0.7, -0.3, 0.25));
  plan = dut.Plan(pose);
  EXPECT_TRUE(plan.has_value());

  // TODO(sammy-tri) It would be good to have a test for planning failures
  // here, but this causes test timeouts under IPOPT (all builds) and SNOPT
  // (debug/asan builds).  If ConstraintRelaxingIk were configurable, we could
  // make it fail quickly enough to add a test here.
}

}  // namespace util
}  // namespace manipulation
}  // namespace drake
