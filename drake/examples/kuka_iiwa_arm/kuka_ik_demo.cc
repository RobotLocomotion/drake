/// @file
///
/// Generates a canned IK demo plan for an iiwa arm starting from the
/// zero configuration and sends that plan over lcm using the
/// robot_plan_t message.

#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/polynomial.h"
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::Vector2d;
using Eigen::Vector3d;

using drake::Vector1d;

const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";

int main(int argc, const char* argv[]) {
  RigidBodyTree tree(
      drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      drake::systems::plants::joints::kFixed);

  // Create a basic pointwise IK trajectory for moving the iiwa arm.
  // We start in the zero configuration (straight up).

  // TODO(sam.creasey) We should start planning with the robot's
  // current position rather than assuming vertical.
  VectorXd zero_conf = tree.getZeroConfiguration();
  VectorXd joint_lb = zero_conf - VectorXd::Constant(7, 0.01);
  VectorXd joint_ub = zero_conf + VectorXd::Constant(7, 0.01);

  PostureConstraint pc1(&tree, Vector2d(0, 0.5));
  VectorXi joint_idx(7);
  joint_idx << 0, 1, 2, 3, 4, 5, 6;
  pc1.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Define an end effector constraint and make it active for the
  // timespan from 1 to 3 seconds.
  Vector3d pos_end;
  pos_end << 0.6, 0, 0.325;
  Vector3d pos_lb = pos_end - Vector3d::Constant(0.005);
  Vector3d pos_ub = pos_end + Vector3d::Constant(0.005);
  WorldPositionConstraint wpc(&tree, tree.FindBodyIndex("iiwa_link_ee"),
                              Vector3d::Zero(), pos_lb, pos_ub, Vector2d(1, 3));

  // After the end effector constraint is released, apply the straight
  // up configuration again.
  PostureConstraint pc2(&tree, Vector2d(4, 5.9));
  pc2.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Bring back the end effector constraint through second 9 of the
  // demo.
  WorldPositionConstraint wpc2(&tree, tree.FindBodyIndex("iiwa_link_ee"),
                               Vector3d::Zero(), pos_lb, pos_ub,
                               Vector2d(6, 9));

  // For part of the remaining time, constrain the second joint while
  // preserving the end effector constraint.
  //
  // Variable `joint_position_start_idx` below is a collection of offsets into
  // the state vector referring to the positions of the joints to be
  // constrained.
  Eigen::VectorXi joint_position_start_idx(1);
  joint_position_start_idx(0) = tree.FindChildBodyOfJoint("iiwa_joint_2")->
      get_position_start_index();
  PostureConstraint pc3(&tree, Vector2d(6, 8));
  pc3.setJointLimits(joint_position_start_idx, Vector1d(0.7), Vector1d(0.8));


  const int kNumTimesteps = 5;
  double t[kNumTimesteps] = { 0.0, 2.0, 5.0, 7.0, 9.0 };
  MatrixXd q0(tree.get_num_positions(), kNumTimesteps);
  for (int i = 0; i < kNumTimesteps; i++) {
    q0.col(i) = zero_conf;
  }

  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back(&pc1);
  constraint_array.push_back(&wpc);
  constraint_array.push_back(&pc2);
  constraint_array.push_back(&pc3);
  constraint_array.push_back(&wpc2);
  IKoptions ikoptions(&tree);
  int info[kNumTimesteps];
  MatrixXd q_sol(tree.get_num_positions(), kNumTimesteps);
  std::vector<std::string> infeasible_constraint;

  inverseKinPointwise(&tree, kNumTimesteps, t, q0, q0, constraint_array.size(),
                      constraint_array.data(), ikoptions, &q_sol, info,
                      &infeasible_constraint);
  bool info_good = true;
  for (int i = 0; i < kNumTimesteps; ++i) {
    printf("INFO[%d] = %d ", i, info[i]);
    if (info[i] != 1) {
      info_good = false;
    }
  }
  printf("\n");

  if (!info_good) {
    std::cerr << "Solution failed, not sending." << std::endl;
    return 1;
  }

  robotlocomotion::robot_plan_t plan{};
  plan.utime = 0;  // I (sam.creasey) don't think this is used?
  plan.robot_name = "iiwa";  // Arbitrary, probably ignored
  plan.num_states = kNumTimesteps;
  const bot_core::robot_state_t default_robot_state{};
  plan.plan.resize(kNumTimesteps, default_robot_state);
  plan.plan_info.resize(kNumTimesteps, 0);
  /// Encode the q_sol returned for each timestep into the vector of
  /// robot states.
  for (int i = 0; i < kNumTimesteps; i++) {
    bot_core::robot_state_t& step = plan.plan[i];
    step.utime = t[i] * 1e6;
    step.num_joints = 7;
    for (int j = 0; j < step.num_joints; j++) {
      step.joint_name.push_back(tree.get_position_name(j));
      step.joint_position.push_back(q_sol(j, i));
      step.joint_velocity.push_back(0);
      step.joint_effort.push_back(0);
    }
    plan.plan_info[i] = info[i];
  }
  plan.num_grasp_transitions = 0;
  plan.left_arm_control_type = plan.POSITION;
  plan.right_arm_control_type = plan.NONE;
  plan.left_leg_control_type = plan.NONE;
  plan.right_leg_control_type = plan.NONE;
  plan.num_bytes = 0;

  lcm::LCM lcm;
  return lcm.publish(kLcmPlanChannel, &plan);
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main(int argc, const char* argv[]) {
  return drake::examples::kuka_iiwa_arm::main(argc, argv);
}
