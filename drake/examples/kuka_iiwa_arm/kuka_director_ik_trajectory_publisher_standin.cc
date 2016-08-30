#include <iostream>

#include <lcm/lcm-cpp.hpp>

// #include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
// #include "drake/common/polynomial.h"
// #include "drake/systems/plants/IKoptions.h"
// #include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"
// #include "drake/systems/plants/constraint/RigidBodyConstraint.h"
// #include "drake/systems/trajectories/PiecewisePolynomial.h"
// #include "drake/systems/vector.h"

// #include "lcmtypes/drake/lcmt_iiwa_command.hpp"
// #include "lcmtypes/drake/lcmt_iiwa_status.hpp"
#include "lcmtypes/drake_robot_plan_t.h"

// #include "iiwa_status.h"

// using Eigen::MatrixXd;
// using Eigen::VectorXd;
// using Eigen::VectorXi;
// using drake::Vector1d;
// using Eigen::Vector2d;
// using Eigen::Vector3d;

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
  lcm::LCM lcm;

  RigidBodyTree tree(
      drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      DrakeJoint::FIXED);

  


  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main(int argc, const char* argv[]) {
  return drake::examples::kuka_iiwa_arm::DoMain(argc, argv);
}
