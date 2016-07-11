#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "drake/Path.h"
#include "drake/core/Vector.h"
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"

#include "lcmtypes/drake/lcmt_iiwa_command.hpp"
#include "lcmtypes/drake/lcmt_iiwa_status.hpp"

#include "iiwa_status.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* kLcmCommandChannel = "IIWA_COMMAND";

/// This is a really simple demo class to run a trajectory which is
/// the output of an IK plan.  It lacks a lot of useful things, like a
/// controller which does a remotely good job of mapping the
/// trajectory onto the robot.  The paramaters @p nT and @p t are
/// identical to their usage for inverseKinPointwise (@p nT is number
/// of time samples and @p t is an array of times in seconds).
class TrajectoryRunner {
 public:
  TrajectoryRunner(std::shared_ptr<lcm::LCM> lcm, int nT, const double* t,
                   const Eigen::MatrixXd& traj)
      : lcm_(lcm), nT_(nT), t_(t), traj_(traj) {
    lcm_->subscribe(IiwaStatus<double>::channel(),
                    &TrajectoryRunner::HandleStatus, this);
  }

  void Run() {
    int64_t start_time = -1;
    int cur_step = 0;

    // Don't try to run an instantaneous command at time zero.  It
    // won't make any sense.
    if (t_[cur_step] == 0.0) {
      cur_step++;
    }

    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = kNumJoints;
    iiwa_command.joint_position.resize(kNumJoints, 0.);

    while (cur_step < nT_) {
      int handled  = lcm_->handleTimeout(10);  // timeout is in msec -
                                               // should be safely
                                               // bigger than e.g. a
                                               // 200Hz input rate
      if (handled <= 0) {
        std::cerr << "Failed to receive LCM status." << std::endl;
        return;
      }

      if (start_time == -1) {
        start_time = iiwa_status_.timestamp;
      }

      const auto desired_next = traj_.col(cur_step);

      iiwa_command.timestamp = iiwa_status_.timestamp;

      // This is totally arbitrary.  There's no good reason to
      // implement this as a maximum delta to submit per tick.  What
      // we actually need is something like a proper
      // planner/interpolater which spreads the motion out over the
      // entire duration from current_t to next_t, and commands the
      // next position taking into account the velocity of the joints
      // and the distance remaining.
      const double max_joint_delta = 0.1;
      for (int joint = 0; joint < kNumJoints; joint++) {
        double joint_delta =
            desired_next[joint] - iiwa_status_.joint_position_measured[joint];
        joint_delta = std::max(-max_joint_delta,
                               std::min(max_joint_delta, joint_delta));
        iiwa_command.joint_position[joint] =
            iiwa_status_.joint_position_measured[joint] + joint_delta;
      }

      lcm_->publish(kLcmCommandChannel, &iiwa_command);
      if ((iiwa_status_.timestamp - start_time) / 1e3 > t_[cur_step]) {
        cur_step++;
      }
    }
  }

 private:
  void HandleStatus(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                    const lcmt_iiwa_status* status) {
    iiwa_status_ = *status;
  }

  static const int kNumJoints = 7;
  std::shared_ptr<lcm::LCM> lcm_;
  const int nT_;
  const double* t_;
  const Eigen::MatrixXd& traj_;
  lcmt_iiwa_status iiwa_status_;
};

int do_main(int argc, const char* argv[]) {
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  RigidBodyTree tree(
      Drake::getDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      DrakeJoint::FIXED);

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
  Eigen::VectorXi joint_idx_3(1);
  joint_idx_3(0) = tree.findJoint("iiwa_joint_2")->position_num_start;
  PostureConstraint pc3(&tree, Vector2d(6, 8));
  pc3.setJointLimits(joint_idx_3, Vector1d(0.63), Vector1d(0.7));


  const int kNumTimesteps = 5;
  double t[kNumTimesteps] = { 0.0, 2.0, 5.0, 7.0, 9.0 };
  MatrixXd q0(tree.number_of_positions(), kNumTimesteps);
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
  MatrixXd q_sol(tree.number_of_positions(), kNumTimesteps);
  std::vector<std::string> infeasible_constraint;

  inverseKinPointwise(&tree, kNumTimesteps, t, q0, q0, constraint_array.size(),
                      constraint_array.data(), ikoptions, &q_sol, info,
                      &infeasible_constraint);
  bool info_good = true;
  for (int i = 0; i < kNumTimesteps; i++) {
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

  // Now run through the plan.
  TrajectoryRunner runner(lcm, kNumTimesteps, t, q_sol);
  runner.Run();
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main(int argc, const char* argv[]) {
  return drake::examples::kuka_iiwa_arm::do_main(argc, argv);
}
