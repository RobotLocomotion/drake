#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/robot_plan_t.hpp"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

// const char* kLcmChannelName = "COMMITTED_ROBOT_PLAN";

/**
 * Generates an joint-space trajectory for the Kuka IIWA robot, saves this
 * trajectory inside a robot_plan_t LCM message, and then publishes this LCM
 * on LCM channel kLcmChannelName.
 */
int DoMain(int argc, const char* argv[]) {
  // Waits for the user to type a key.
  std::cout << "Please press any key to continue..." << std::endl;
  getchar();

  // Instantiates a RigidBodyTree containing an IIWA robot instance.
  std::shared_ptr<RigidBodyTree> tree(new RigidBodyTree(
      drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      DrakeJoint::FIXED));

  // Generates the joint space trajectory.
  std::cout << "Generating joint space trajectory..." << std::endl;

  // Saves the joint space trajectory in to an LCM message.
  std::cout << "Saving joint space trajectory into message..." << std::endl;
  // TODO

  // Publish the LCM message.
  lcm::LCM lcm;
  // lcm.publish()

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main(int argc, const char* argv[]) {
  return drake::examples::kuka_iiwa_arm::DoMain(argc, argv);
}
