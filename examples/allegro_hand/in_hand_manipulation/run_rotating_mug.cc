/// @file
///
/// This file set up a control procedure of using the Allegro hand to rotate or
/// translate the mug in controlled directions. The program currently runs only
/// in simulation, together with the allegro_single_object_simulation.cc

#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/examples/allegro_hand/allegro_common.h"
#include "drake/examples/allegro_hand/allegro_lcm.h"
#include "drake/examples/allegro_hand/in_hand_manipulation/mug_state_set.h"
#include "drake/lcmt_allegro_command.hpp"
#include "drake/lcmt_allegro_status.hpp"
#include "drake/manipulation/planner/differential_inverse_kinematics.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"

#include "robotlocomotion/pose_t.hpp"

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>

namespace drake {
namespace examples {
namespace allegro_hand {
namespace {

using drake::manipulation::planner::DifferentialInverseKinematicsResult;
using drake::manipulation::planner::DifferentialInverseKinematicsParameters;
using drake::manipulation::planner::DoDifferentialInverseKinematics;
using drake::manipulation::planner::DifferentialInverseKinematicsStatus;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::Body;
using drake::multibody::Frame;

const char* const kLcmStatusChannel = "ALLEGRO_STATUS";
const char* const kLcmCommandChannel = "ALLEGRO_COMMAND";
const char* const kLcmObjTargetChannel = "TARGET_OBJ_POSE";

class HandPositionCommander {
 public:
  HandPositionCommander() {
    lcm_.subscribe(kLcmStatusChannel, &HandPositionCommander::HandleStatus,
                   this);
    lcm_.subscribe(kLcmObjTargetChannel,
                   &HandPositionCommander::TargetFrameInput, this);

    // Load plant of the hand. This program only runs for the right hand
    plant_ = std::make_unique<MultibodyPlant<double>>(1e-3);
    const std::string HandPath = FindResourceOrThrow(
        "drake/manipulation/"
        "models/allegro_hand_description/sdf/"
        "allegro_hand_description_right.sdf");
    multibody::parsing::AddModelFromSdfFile(HandPath, plant_.get());
    const auto& joint_hand_root = plant_->GetBodyByName("hand_root");
    plant_->AddJoint<multibody::WeldJoint>("weld_hand", plant_->world_body(),
                                           {}, joint_hand_root, {},
                                           Isometry3<double>::Identity());
    plant_->Finalize();
    plant_context_ = plant_->CreateDefaultContext();

    // Ini the port mapping matril Px for Allegro Hand. Px_half is the
    // submatrix for only the position mapping
    MatrixX<double> Px, Py;
    GetControlPortMapping(*plant_, &Px, &Py);
    Px_half = Px.block(0, 0, 16, 16);

    updating_target_frame_ = false;
    mug_pose_.matrix().setIdentity();
    saved_target.resize(4);
    target_fingertip_frame_.resize(4);

    // Ini command for Allegro hand
    allegro_command_.num_joints = kAllegroNumJoints;
    allegro_command_.joint_position.resize(kAllegroNumJoints, 0.);
    allegro_command_.num_torques = 0;
    allegro_command_.joint_torque.resize(0);
  }

  void run_close_hand() {
    Eigen::VectorXd target_joint_pose(kAllegroNumJoints);
    target_joint_pose.setZero();
    target_joint_pose(0) = 1.396;
    target_joint_pose(1) = 0.3;
    // move to the position of opening the hand
    MovetoPositionUntilStuck(target_joint_pose);

    // grasp on points
    mug_state_.GetGraspTargetFrames(mug_pose_, &target_fingertip_frame_,
                                    &target_frame_for_differentialIK_);
    iniIKtarget();
    updating_target_frame_ = true;
    KeepMovingUntilStuck(200);
    updating_target_frame_ = false;
  }

  /// Rotating the mug along X axis repeatively, for @parm rotation_angle
  /// angle. Must be called after the mug is in gripping
  void run_rotate_X(double rotation_angle) {
    mug_state_.UpdateMugPose(mug_pose_);
    mug_state_.GetXRotatedTargetFrame(rotation_angle / 180.0 * M_PI,
                                      &target_fingertip_frame_);
    iniIKtarget();
    std::cout << "Rotating in one direction \n";
    KeepMovingUntilStuck(2000);

    while (true) {
      mug_state_.UpdateMugPose(mug_pose_);
      mug_state_.GetXRotatedTargetFrame(-rotation_angle / 90.0 * M_PI,
                                        &target_fingertip_frame_);
      iniIKtarget();
      std::cout << "Rotating in one direction \n";
      KeepMovingUntilStuck(2000);

      mug_state_.UpdateMugPose(mug_pose_);
      mug_state_.GetXRotatedTargetFrame(rotation_angle / 90.0 * M_PI,
                                        &target_fingertip_frame_);
      iniIKtarget();
      std::cout << "Rotating in one direction \n";
      KeepMovingUntilStuck(2000);
    }
  }

  /// Rotating the mug along Y axis repeatively, for @parm rotation_angle
  /// angle. Must be called after the mug is in gripping
  void run_rotate_Y(double rotation_angle) {
    mug_state_.UpdateMugPose(mug_pose_);
    mug_state_.GetYRotatedTargetFrame(rotation_angle / 180.0 * M_PI,
                                      &target_fingertip_frame_);
    iniIKtarget();
    std::cout << "Rotating in one direction \n";
    KeepMovingUntilStuck(2000);

    while (true) {
      mug_state_.UpdateMugPose(mug_pose_);
      mug_state_.GetYRotatedTargetFrame(-rotation_angle / 90.0 * M_PI,
                                        &target_fingertip_frame_);
      iniIKtarget();
      std::cout << "Rotating in one direction \n";
      KeepMovingUntilStuck(2000);

      mug_state_.UpdateMugPose(mug_pose_);
      mug_state_.GetYRotatedTargetFrame(rotation_angle / 90.0 * M_PI,
                                        &target_fingertip_frame_);
      iniIKtarget();
      std::cout << "Rotating in one direction \n";
      KeepMovingUntilStuck(2000);
    }
  }

  /// Rotating the mug along Z axis repeatively, for @parm rotation_angle
  /// angle. Must be called after the mug is in gripping
  void run_rotate_Z(double rotation_angle) {
    mug_state_.UpdateMugPose(mug_pose_);
    mug_state_.GetZRotatedTargetFrame(rotation_angle / 180.0 * M_PI,
                                      &target_fingertip_frame_);
    iniIKtarget();
    std::cout << "Rotating in one direction \n";
    KeepMovingUntilStuck(2000);

    while (true) {
      mug_state_.UpdateMugPose(mug_pose_);
      mug_state_.GetZRotatedTargetFrame(-rotation_angle / 90.0 * M_PI,
                                        &target_fingertip_frame_);
      iniIKtarget();
      std::cout << "Rotating in one direction \n";
      KeepMovingUntilStuck(2000);

      mug_state_.UpdateMugPose(mug_pose_);
      mug_state_.GetZRotatedTargetFrame(rotation_angle / 90.0 * M_PI,
                                        &target_fingertip_frame_);
      iniIKtarget();
      std::cout << "Rotating in one direction \n";
      KeepMovingUntilStuck(2000);
    }
  }

  /// Translate the mug position in the space
  void run_translation() {
    mug_state_.UpdateMugPose(mug_pose_);
    mug_state_.GetTransTargetFrame(Eigen::Vector3d(-0.015, -0.006, 0.006),
                                   &target_fingertip_frame_);
    iniIKtarget();
    std::cout << "Translating in one direction \n";
    KeepMovingUntilStuck(2000);

    while (true) {
      mug_state_.UpdateMugPose(mug_pose_);
      mug_state_.GetTransTargetFrame(Eigen::Vector3d(0.012, 0.012, -0.012),
                                     &target_fingertip_frame_);
      iniIKtarget();
      std::cout << "Translating in one direction \n";
      KeepMovingUntilStuck(2000);

      mug_state_.UpdateMugPose(mug_pose_);
      mug_state_.GetTransTargetFrame(Eigen::Vector3d(-0.012, -0.012, 0.012),
                                     &target_fingertip_frame_);
      iniIKtarget();
      std::cout << "Translating in one direction \n";
      KeepMovingUntilStuck(2000);
    }
  }

 private:
  inline void PublishPositionCommand(const Eigen::VectorXd target_joint_pose) {
    Eigen::VectorXd::Map(&allegro_command_.joint_position[0],
                         kAllegroNumJoints) = target_joint_pose;
    lcm_.publish(kLcmCommandChannel, &allegro_command_);
  }

  inline void KeepMovingUntilStuck(int count = 40) {
    // the first loop serves as a time delay to filter out the noise in the
    // hand's motion.
    for (int i = 0; i < count; i++) {
      while (0 == lcm_.handleTimeout(10) || allegro_status_.utime == -1) {
      }
    }
    flag_hand_is_moving_ = true;
    while (flag_hand_is_moving_) {
      while (0 == lcm_.handleTimeout(10) || allegro_status_.utime == -1) {
      }
    }
  }

  inline void MovetoPositionUntilStuck(
      const Eigen::VectorXd target_joint_pose) {
    PublishPositionCommand(target_joint_pose);
    KeepMovingUntilStuck(100);
  }

  /// Using IK to find the joint positions so that the fingertips will reach
  /// target_fingertip_frame_, and send the command for the joint positions
  /// through LCM
  void iniIKtarget(double target_tor = 5e-3) {
    multibody::InverseKinematics ik_(*plant_);
    const Frame<double>& WorldFrame = plant_->world_frame();

    Eigen::Vector3d p_W_tor = Eigen::Vector3d::Constant(target_tor);
    Eigen::Vector3d p_W;
    Isometry3<double> fingertip_offset;
    fingertip_offset.matrix().setIdentity();
    Isometry3<double> finger_target_transfer;
    fingertip_offset.translation() = Eigen::Vector3d(0, 0, 0.012);
    for (unsigned int cur_finger = 0; cur_finger < 4; cur_finger++) {
      Eigen::Vector3d p_TipFinger(0, 0, 0.0267);
      const Frame<double>* fingertip_frame{nullptr};
      if (cur_finger == 0) {  // if it's the thumb
        p_TipFinger(2) = 0.0423;
        fingertip_frame = &(plant_->GetFrameByName("link_15"));
      } else {
        fingertip_frame = &(plant_->GetFrameByName(
            "link_" + std::to_string(cur_finger * 4 - 1)));
      }

      finger_target_transfer =
          target_fingertip_frame_[cur_finger] * fingertip_offset;
      p_W = finger_target_transfer.translation();
      saved_target[cur_finger] = finger_target_transfer;
      ik_.AddPositionConstraint(*fingertip_frame, p_TipFinger, WorldFrame,
                                p_W - p_W_tor, p_W + p_W_tor);
    }

    const auto result = ik_.get_mutable_prog()->Solve();
    std::cout << "Did IK find result? " << result << std::endl;
    const auto q_sol = ik_.prog().GetSolution(ik_.q());
    saved_joint_command = q_sol;
    SendJointCommand();
  }

  /// Using differential IK to update the target joint positions of the fingers
  /// so that the fingertips will reach target_frame_for_differentialIK_, and
  /// send the command for the joint positions through LCM
  void updateIKtarget() {
    // update context of the plant
    plant_->tree()
        .get_mutable_multibody_state_vector(plant_context_.get())
        .head(16) = saved_joint_command;

    // set parameters for differential IK
    std::unique_ptr<DifferentialInverseKinematicsParameters> params_ =
        std::make_unique<DifferentialInverseKinematicsParameters>(16, 16);
    params_->set_nominal_joint_position(saved_joint_command);
    params_->set_unconstrained_degrees_of_freedom_velocity_limit(0.6);
    params_->set_timestep(1e-3);
    // This is only an approximate boundary for the joint positions
    std::pair<VectorX<double>, VectorX<double>> q_bounds = {
        VectorX<double>::Constant(16, -0.5),
        VectorX<double>::Constant(16, 1.6)};
    std::pair<VectorX<double>, VectorX<double>> v_bounds = {
        VectorX<double>::Constant(16, -7), VectorX<double>::Constant(16, 7)};
    params_->set_joint_position_limits(q_bounds);
    params_->set_joint_velocity_limits(v_bounds);

    Eigen::Vector3d p_W;
    bool target_updated_flag = false;
    Isometry3<double> fingertip_offset;
    fingertip_offset.matrix().setIdentity();
    fingertip_offset.translation() += Eigen::Vector3d(0, 0, 0.012);
    for (unsigned int cur_finger = 0; cur_finger < 4; cur_finger++) {
      // if the new position is similar to the saved one, don't need to update
      Isometry3<double> finger_target_transfer =
          mug_pose_ * target_frame_for_differentialIK_[cur_finger] *
          fingertip_offset;
      if (saved_target[cur_finger].isApprox(finger_target_transfer)) continue;

      Eigen::Vector3d p_TipFinger(0, 0, 0.0267);
      const Frame<double>* fingertip_frame{nullptr};
      if (cur_finger == 0) {  // if it's the thumb
        p_TipFinger(2) = 0.0423;
        fingertip_frame = &(plant_->GetFrameByName("link_15"));
      } else {
        fingertip_frame = &(plant_->GetFrameByName(
            "link_" + std::to_string(cur_finger * 4 - 1)));
      }
      Vector6<double> V_WE_desired =
          manipulation::planner::ComputePoseDiffInCommonFrame(
              saved_target[cur_finger], finger_target_transfer);
      MatrixX<double> J_WE(6, 16);
      plant_->tree().CalcFrameGeometricJacobianExpressedInWorld(
          *plant_context_, *fingertip_frame, Vector3<double>::Zero(), &J_WE);

      DifferentialInverseKinematicsResult mbt_result =
          DoDifferentialInverseKinematics(saved_joint_command,
                                          Eigen::VectorXd::Zero(16),
                                          V_WE_desired, J_WE, *params_);
      if (mbt_result.status ==
          DifferentialInverseKinematicsStatus::kSolutionFound) {
        saved_joint_command += mbt_result.joint_velocities.value();
        saved_target[cur_finger] = finger_target_transfer;
        target_updated_flag = true;
      }
    }
    if (target_updated_flag) SendJointCommand();
  }

  void SendJointCommand() {
    Eigen::VectorXd::Map(&allegro_command_.joint_position[0],
                         kAllegroNumJoints) = Px_half * saved_joint_command;
    lcm_.publish(kLcmCommandChannel, &allegro_command_);
  }

  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_allegro_status* status) {
    allegro_status_ = *status;
    hand_state.Update(allegro_status_);
    flag_hand_is_moving_ = !hand_state.IsAllFingersStuck();
  }

  void TargetFrameInput(const ::lcm::ReceiveBuffer*, const std::string&,
                        const robotlocomotion::pose_t* msg_mug_pose_) {
    // update the pose of the mug
    mug_pose_.matrix().setIdentity();
    mug_pose_.translation() << msg_mug_pose_->position.x,
        msg_mug_pose_->position.y, msg_mug_pose_->position.z;
    mug_pose_.rotate(Eigen::Quaternion<double>(
        msg_mug_pose_->orientation.w, msg_mug_pose_->orientation.x,
        msg_mug_pose_->orientation.y, msg_mug_pose_->orientation.z));

    // display the target points for on the mug
    mug_state_.PublishTargetFrametoLcm(mug_pose_);

    if (updating_target_frame_) updateIKtarget();
  }

  ::lcm::LCM lcm_;
  lcmt_allegro_status allegro_status_;
  lcmt_allegro_command allegro_command_;
  AllegroHandMotionState hand_state;

  bool flag_hand_is_moving_ = true;
  bool updating_target_frame_ = false;

  // params for the hand plant
  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<systems::Context<double>> plant_context_;
  MatrixX<double> Px_half;
  std::vector<Isometry3<double>> saved_target;
  // The current joint position targets that has been sent through LCM
  Eigen::VectorXd saved_joint_command;

  // control part for the target finger positions
  SetMugStateControl mug_state_;
  Isometry3<double> mug_pose_;
  std::vector<Isometry3<double>> target_fingertip_frame_;
  std::vector<Isometry3<double>> target_frame_for_differentialIK_;
};

int do_main() {
  HandPositionCommander runner;
  runner.run_close_hand();
  // test moving the mug into different poses. Could choose from run_rotate_Y,
  // run_rotate_Z, or run_translation
  runner.run_rotate_X(8.0);
  return 0;
}

}  // namespace
}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake

int main() { 
  return drake::examples::allegro_hand::do_main(); 
}
