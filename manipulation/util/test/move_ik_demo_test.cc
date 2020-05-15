#include "drake/manipulation/util/move_ik_demo.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"

namespace drake {
namespace manipulation {
namespace util {

const char kIiwaUrdf[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_no_collision.urdf";

GTEST_TEST(MoveIkDemoTest, IiwaTest) {
  math::RigidTransformd pose(
      math::RollPitchYawd(0, 0, -1.57),
      Eigen::Vector3d(0.8, -0.3, 0.25));

  MoveIkDemo dut(
      FindResourceOrThrow(kIiwaUrdf),
      "base", "iiwa_link_ee", pose, 100);
  dut.set_joint_velocity_limits(kuka_iiwa::get_iiwa_max_joint_velocities());
  auto plan = dut.HandleStatus(Eigen::VectorXd::Ones(7));
  EXPECT_TRUE(plan.has_value());
  plan = dut.HandleStatus(Eigen::VectorXd::Ones(7));
  EXPECT_FALSE(plan.has_value());
}

}  // namespace util
}  // namespace manipulation
}  // namespace drake
