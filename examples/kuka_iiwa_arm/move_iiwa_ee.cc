/// @file
///
/// Demo of moving the iiwa's end effector in cartesian space.  This
/// program creates a plan to move the end effector from the current
/// position to the location specified on the command line.  The
/// current calculated position of the end effector is printed before,
/// during, and after the commanded motion.

#include <iostream>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_robot_plan.hpp"
#include "drake/manipulation/util/move_ik_demo_base.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/package_map.h"

DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_string(lcm_status_channel, "IIWA_STATUS",
              "Channel on which to listen for lcmt_iiwa_status messages.");
DEFINE_string(lcm_plan_channel, "COMMITTED_ROBOT_PLAN",
              "Channel on which to send lcmt_robot_plan messages.");
DEFINE_double(x, 0., "x coordinate to move to");
DEFINE_double(y, 0., "y coordinate to move to");
DEFINE_double(z, 0., "z coordinate to move to");
DEFINE_double(roll, 0., "target roll about world x axis for end effector");
DEFINE_double(pitch, 0., "target pitch about world y axis for end effector");
DEFINE_double(yaw, 0., "target yaw about world z axis for end effector");
DEFINE_string(ee_name, "iiwa_link_ee", "Name of the end effector link");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

using lcm::DrakeLcm;
using lcm::Subscriber;
using manipulation::util::MoveIkDemoBase;
using multibody::PackageMap;

const char kIiwaUrdfUrl[] =
    "package://drake_models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

int DoMain() {
  math::RigidTransformd pose(
      math::RollPitchYawd(FLAGS_roll, FLAGS_pitch, FLAGS_yaw),
      Eigen::Vector3d(FLAGS_x, FLAGS_y, FLAGS_z));

  MoveIkDemoBase demo(
      !FLAGS_urdf.empty() ? FLAGS_urdf : PackageMap{}.ResolveUrl(kIiwaUrdfUrl),
      "base", FLAGS_ee_name, 100);
  demo.set_joint_velocity_limits(get_iiwa_max_joint_velocities());

  // Wait for a status message.
  DrakeLcm lcm;
  Subscriber<lcmt_iiwa_status> subscriber(&lcm, FLAGS_lcm_status_channel);
  while (subscriber.count() == 0) {
    lcm.HandleSubscriptions(10);
  }

  // Read the positions.
  const lcmt_iiwa_status& status = subscriber.message();
  Eigen::VectorXd iiwa_q(status.num_joints);
  for (int i = 0; i < status.num_joints; i++) {
    iiwa_q[i] = status.joint_position_measured[i];
  }

  // Make a plan.
  demo.HandleStatus(iiwa_q);
  std::optional<lcmt_robot_plan> plan = demo.Plan(pose);
  if (!plan.has_value()) {
    std::cerr << "Plan failed; exiting!\n";
    return 0;
  }

  // Run the plan.
  Publish(&lcm, FLAGS_lcm_plan_channel, plan.value());
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::DoMain();
}
