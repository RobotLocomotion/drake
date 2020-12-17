/// @file
///
/// This file set up an example about control the allegro hand based on
/// position. In the program, the hand firstly grasps on a mug, and then twsits
/// it repeatedly. The program presently only runs on simulation, with the file
/// allegro_single_object_simulation.cc which creates the simulation environment
/// for the hand and object. This program reads from LCM about the state of the
/// hands, and process command the positions of the finger joints through LCM.
/// It also uses the velocity states of the fingers to decide whether the hand
/// has finished the current motion, either by reaching the target position or
/// get stuck by collisions.

#include "lcm/lcm-cpp.hpp"
#include <Eigen/Dense>
#include <gflags/gflags.h>

#include "drake/examples/allegro_hand/allegro_common.h"
#include "drake/examples/allegro_hand/allegro_lcm.h"
#include "drake/lcmt_allegro_command.hpp"
#include "drake/lcmt_allegro_status.hpp"

DEFINE_int32(max_cycles, 1000'000'000, "Stop after this many twists.");

namespace drake {
namespace examples {
namespace allegro_hand {
namespace {

const char* const kLcmStatusChannel = "ALLEGRO_STATUS";
const char* const kLcmCommandChannel = "ALLEGRO_COMMAND";

class PositionCommander {
 public:
  PositionCommander() {
    lcm_.subscribe(kLcmStatusChannel, &PositionCommander::HandleStatus,
                   this);
  }

  void Run() {
    allegro_command_.num_joints = kAllegroNumJoints;
    allegro_command_.joint_position.resize(kAllegroNumJoints, 0.);
    allegro_command_.num_torques = 0;
    allegro_command_.joint_torque.resize(0);

    flag_moving = true;
    Eigen::VectorXd target_joint_position(kAllegroNumJoints);
    target_joint_position.setZero();
    MovetoPositionUntilStuck(target_joint_position);

    // close thumb
    target_joint_position(0) = 1.396;
    target_joint_position(1) = 0.3;
    MovetoPositionUntilStuck(target_joint_position);

    // close other fingers
    target_joint_position.segment<4>(0) =
        hand_state_.FingerGraspJointPosition(0);
    target_joint_position.segment<4>(4) =
        hand_state_.FingerGraspJointPosition(1);
    target_joint_position.segment<4>(8) =
        hand_state_.FingerGraspJointPosition(2);
    target_joint_position.segment<4>(12) =
        hand_state_.FingerGraspJointPosition(3);
    MovetoPositionUntilStuck(target_joint_position);
    std::cout << "Hand is closed. \n";
    while (0 == lcm_.handleTimeout(10)) {
    }

    // Record the joint position q when the fingers are close and gripping the
    // object
    Eigen::VectorXd close_hand_joint_position = Eigen::Map<Eigen::VectorXd>(
        &(allegro_status_.joint_position_measured[0]), kAllegroNumJoints);
    // twisting the cup repeatedly
    for (int cycle = 0; cycle < FLAGS_max_cycles; ++cycle) {
      target_joint_position = close_hand_joint_position;
      // The middle finger works as a pivot finger for the rotation, and exert
      // a little force to maintain the stabilization of the mug, which is
      // realized by adding some extra pushing motion towards the balance
      // position. Eigen::Vector3d(1, 1, 0.5) is a number based on experience
      // to keep the finger position, and 0.1 is the coefficient related to
      // the extra force to apply.
      target_joint_position.segment<3>(9) +=
          (0.1 * Eigen::Vector3d(1, 1, 0.5));
      // The thumb works as another pivot finger, and is expected to exert a
      // large force in order to keep stabilization.
      target_joint_position.segment<4>(0) =
          hand_state_.FingerGraspJointPosition(0);
      // The index finger works as the actuating finger, where (1, 0.3, 0.5) is
      // the portion of the joint motion for actuating the mug rotation, 0.6 is
      // the coefficient to determine how much the rotation should be.
      target_joint_position.segment<3>(5) +=
          (0.6 * Eigen::Vector3d(1, 0.3, 0.5));
      MovetoPositionUntilStuck(target_joint_position);

      target_joint_position = close_hand_joint_position;
      target_joint_position.segment<3>(9) +=
          (0.1 * Eigen::Vector3d(1, 1, 0.5));
      target_joint_position.segment<4>(0) =
          hand_state_.FingerGraspJointPosition(0);
      // The ring finger works as the actuating finger now to rotate the mug in
      // the opposite direction.
      target_joint_position.segment<3>(13) +=
          (0.6 * Eigen::Vector3d(1, 0.3, 0.5));
      MovetoPositionUntilStuck(target_joint_position);
    }
  }

 private:
  void PublishPositionCommand(
      const Eigen::VectorXd& target_joint_position) {
    Eigen::VectorXd::Map(&allegro_command_.joint_position[0],
                         kAllegroNumJoints) = target_joint_position;
    lcm_.publish(kLcmCommandChannel, &allegro_command_);
  }

  void MovetoPositionUntilStuck(
      const Eigen::VectorXd& target_joint_position) {
    PublishPositionCommand(target_joint_position);
    // A time delay at the initial moving stage so that the noisy data from the
    // hand motion is filtered.
    for (int i = 0; i < 60; i++) {
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
    hand_state_.Update(allegro_status_);
    flag_moving = !hand_state_.IsAllFingersStuck();
  }

  ::lcm::LCM lcm_;
  lcmt_allegro_status allegro_status_;
  lcmt_allegro_command allegro_command_;
  AllegroHandMotionState hand_state_;

  bool flag_moving = true;
};

int do_main() {
  PositionCommander runner;
  runner.Run();
  return 0;
}

}  // namespace
}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::allegro_hand::do_main();
}
