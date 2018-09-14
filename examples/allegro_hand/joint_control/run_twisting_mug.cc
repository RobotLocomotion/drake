/// @file
///
/// This file set up an example about control the allegro hand based on
/// position. In the program, the hand firstly grasps on a mug, and then twsits
/// it repeatly. The program presently only runs on simulation, with the file
/// allegro_single_object_simulation.cc which creats the simulation environment
/// for the hand and object. This program reads from LCM about the state of the
/// hands, and process command the positions of the finger joints through LCM.
/// It also uses the velocity states of the fingers to decide whether the hand
/// has finished the current motion, either by reaching the target position or
/// get stuck by collisions.

#include <iomanip>

#include <Eigen/Dense>
#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_assert.h"
#include "drake/examples/allegro_hand/allegro_common.h"
#include "drake/examples/allegro_hand/allegro_lcm.h"
#include "drake/lcmt_allegro_command.hpp"
#include "drake/lcmt_allegro_status.hpp"

namespace drake {
namespace examples {
namespace allegro_hand {
namespace {

const char* const kLcmStatusChannel = "ALLEGRO_STATUS";
const char* const kLcmCommandChannel = "ALLEGRO_COMMAND";

class ConstantPositionInput {
 public:
  ConstantPositionInput() {
    lcm_.subscribe(kLcmStatusChannel, &ConstantPositionInput::HandleStatus,
                   this);
  }

  void Run() {
    allegro_command.num_joints = kAllegroNumJoints;
    allegro_command.joint_position.resize(kAllegroNumJoints, 0.);
    allegro_command.num_torques = 0;
    allegro_command.joint_torque.resize(kAllegroNumJoints, 0.);

    flag_moving = true;
    Eigen::VectorXd target_joint_pose(kAllegroNumJoints);
    target_joint_pose.setZero();
    MovetoPositionUntilStuck(target_joint_pose);

    // close thumb
    target_joint_pose(0) = 1.396;
    target_joint_pose(1) = 0.3;
    MovetoPositionUntilStuck(target_joint_pose);

    // close other fingers
    target_joint_pose.segment(0, 4) = hand_state.FingerGraspJointPosition(0);
    target_joint_pose.segment(4, 4) = hand_state.FingerGraspJointPosition(1);
    target_joint_pose.segment(8, 4) = hand_state.FingerGraspJointPosition(2);
    target_joint_pose.segment(12, 4) = hand_state.FingerGraspJointPosition(3);
    MovetoPositionUntilStuck(target_joint_pose);
    sleep(2);

    // twisting the cup repeatly
    while (0 == lcm_.handleTimeout(1000)) {
    }
    Eigen::VectorXd close_finger_pose = Eigen::Map<Eigen::VectorXd>(
        &(allegro_status_.joint_position_measured[0]), kAllegroNumJoints);
    while (true) {
      target_joint_pose = close_finger_pose;
      target_joint_pose.segment(9, 3) += (0.15 * Eigen::Vector3d(1, 1, 0.5));
      target_joint_pose.segment(0, 4) = hand_state.FingerGraspJointPosition(0);
      target_joint_pose.segment(5, 3) += (0.6 * Eigen::Vector3d(1, 0.3, 0.5));
      MovetoPositionUntilStuck(target_joint_pose);

      target_joint_pose = close_finger_pose;
      target_joint_pose.segment(9, 3) += (0.15 * Eigen::Vector3d(1, 1, 0.5));
      target_joint_pose.segment(0, 4) = hand_state.FingerGraspJointPosition(0);
      target_joint_pose.segment(13, 3) += (0.6 * Eigen::Vector3d(1, 0.3, 0.5));
      MovetoPositionUntilStuck(target_joint_pose);
    }
  }

 private:
  inline void PublishPositionCommand(const Eigen::VectorXd target_joint_pose) {
    Eigen::VectorXd::Map(&allegro_command.joint_position[0],
                         kAllegroNumJoints) = target_joint_pose;
    lcm_.publish(kLcmCommandChannel, &allegro_command);
  }

  inline void MovetoPositionUntilStuck(
      const Eigen::VectorXd target_joint_pose) {
    PublishPositionCommand(target_joint_pose);
    for (int i = 0; i < 40; i++) {
      while (0 == lcm_.handleTimeout(10) || allegro_status_.utime == -1) {
      }
    }
    // wait until the fingers are stuck, or stop moving.
    while (flag_moving) {
      while (0 == lcm_.handleTimeout(10) || allegro_status_.utime == -1) {
      }
    }
  }

  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_allegro_status* status) {
    allegro_status_ = *status;
    hand_state.Update(allegro_status_);
    flag_moving = !hand_state.IsAllFingersStuck();
  }

  ::lcm::LCM lcm_;
  lcmt_allegro_status allegro_status_;
  lcmt_allegro_command allegro_command;
  AllegroHandState hand_state;

  bool flag_moving = true;
};

int do_main() {
  ConstantPositionInput runner;
  runner.Run();
  return 0;
}

}  // namespace
}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::allegro_hand::do_main(); }
