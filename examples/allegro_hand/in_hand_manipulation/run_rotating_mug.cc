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

    // Ini the port mapping matrix Px for Allegro Hand. Px_position is the
    // submatrix for only the position mapping
    MatrixX<double> Px, Py;
    GetControlPortMapping(*plant_, &Px, &Py);
    Px_position = Px.block(0, 0, 16, 16);

    updating_target_frame_ = false;
    X_WO_.setIdentity();
    X_W_TipCenter_saved_.resize(4);
    X_WF_target_.resize(4);

    // Ini command for Allegro hand
    allegro_command_.num_joints = kAllegroNumJoints;
    allegro_command_.joint_position.resize(kAllegroNumJoints, 0.);
    allegro_command_.num_torques = 0;
    allegro_command_.joint_torque.resize(0);
  }

  void RunCloseHand() {
    Eigen::VectorXd target_joint_position(kAllegroNumJoints);
    target_joint_position.setZero();
    // set the thumb root joint to the upper limit, so that the thumb is placed
    // under the object
    target_joint_position(0) = 1.396;
    target_joint_position(1) = 0.3;
    // move to the position of opening the hand
    MovetoPositionUntilStuck(target_joint_position);

    // grasp on points
    mug_state_.CalcFingerPoseForGrasp(X_WO_, &X_WF_target_,
                                      &X_WF_target_ForDiffIK_);
    InItIKTarget();
    updating_target_frame_ = true;
    KeepMovingUntilStuck(200);
    updating_target_frame_ = false;
  }

  /// Rotating the mug repeatively, for @parm rotation_angle_deg
  void RunRotate(double rotation_angle_deg, char rotation_axis_name,
                 int round) {
    mug_state_.CalcFingerPoseWithMugRotation(rotation_angle_deg / 180.0 * M_PI,
                                             rotation_axis_name, &X_WF_target_,
                                             X_WO_);
    InItIKTarget();
    std::cout << "Rotating in one direction \n";
    KeepMovingUntilStuck(2000);

    for (int i = 0; i < round; i++) {
      mug_state_.CalcFingerPoseWithMugRotation(
          -rotation_angle_deg * 2 / 180.0 * M_PI, rotation_axis_name,
          &X_WF_target_, X_WO_);
      InItIKTarget();
      std::cout << "Rotating in the opposite direction \n";
      KeepMovingUntilStuck(2000);

      mug_state_.CalcFingerPoseWithMugRotation(
          rotation_angle_deg * 2 / 180.0 * M_PI, rotation_axis_name,
          &X_WF_target_, X_WO_);
      InItIKTarget();
      std::cout << "Rotating in one direction \n";
      KeepMovingUntilStuck(2000);
    }
  }

  /// Translate the mug position in the space
  void RunTranslation(int round) {
    mug_state_.CalcFingerPoseWithMugTranslation(
        Eigen::Vector3d(-0.001, -0.002, 0.002), &X_WF_target_, X_WO_);
    InItIKTarget();
    std::cout << "Translating in one direction \n";
    KeepMovingUntilStuck(2000);

    for (int i = 0; i < round; i++) {
      mug_state_.CalcFingerPoseWithMugTranslation(
          Eigen::Vector3d(0.01, 0.01, -0.01), &X_WF_target_, X_WO_);
      InItIKTarget();
      std::cout << "Translating in the opposite direction \n";
      KeepMovingUntilStuck(2000);

      mug_state_.CalcFingerPoseWithMugTranslation(
          Eigen::Vector3d(-0.01, -0.01, 0.01), &X_WF_target_, X_WO_);
      InItIKTarget();
      std::cout << "Translating in one direction \n";
      KeepMovingUntilStuck(2000);
    }
  }

 private:
  inline void PublishPositionCommand(
      const Eigen::VectorXd target_joint_position) {
    Eigen::VectorXd::Map(&allegro_command_.joint_position[0],
                         kAllegroNumJoints) = target_joint_position;
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
      const Eigen::VectorXd target_joint_position) {
    PublishPositionCommand(target_joint_position);
    KeepMovingUntilStuck(100);
  }

  /// Using IK to find the joint positions so that the fingertips will reach
  /// X_WF_target_, and send the command for the joint positions
  /// through LCM
  /// @param target_tor: tolerance of the IK calculation, unit m
  void InItIKTarget(double target_tor = 5e-3) {
    multibody::InverseKinematics ik_(*plant_);
    const Frame<double>& WorldFrame = plant_->world_frame();

    Eigen::Vector3d p_WTipcenter_tor = Eigen::Vector3d::Constant(target_tor);
    Eigen::Vector3d p_WTipcenter;
    Isometry3<double> p_FTipcenter;
    p_FTipcenter.setIdentity();
    Isometry3<double> X_W_TipCenter;
    // 0.012 is the radis of the sphere on the fingertip. In the planning, we
    // consider the center of the fingertip sphere as the origin of the
    // fingertip frame, and this offset ensured the sphere shaped fingertip
    // would reach the target position, no matter where is the contact point on
    // the fingertip
    p_FTipcenter.translation() = Eigen::Vector3d(0, 0, 0.012);
    for (int cur_finger = 0; cur_finger < 4; cur_finger++) {
      Eigen::Vector3d p_Finger_TipCenter(0, 0, 0.0267);
      const Frame<double>* FingertipFrame{nullptr};
      if (cur_finger == 0) {  // if it's the thumb
        // the offset of the tip center on the thumb is different
        p_Finger_TipCenter(2) = 0.0423;
        FingertipFrame = &(plant_->GetFrameByName("link_15"));
      } else {
        FingertipFrame = &(plant_->GetFrameByName(
            "link_" + std::to_string(cur_finger * 4 - 1)));
      }

      X_W_TipCenter = X_WF_target_[cur_finger] * p_FTipcenter;
      p_WTipcenter = X_W_TipCenter.translation();
      X_W_TipCenter_saved_[cur_finger] = X_W_TipCenter;
      ik_.AddPositionConstraint(
          *FingertipFrame, p_Finger_TipCenter, WorldFrame,
          p_WTipcenter - p_WTipcenter_tor, p_WTipcenter + p_WTipcenter_tor);
    }

    const auto result = ik_.get_mutable_prog()->Solve();
    std::cout << "Did IK find result? " << result << std::endl;
    const auto q_sol = ik_.prog().GetSolution(ik_.q());
    q_command_saved_ = q_sol;
    SendJointCommand();
  }

  /// Using differential IK to update the target joint positions of the fingers
  /// so that the fingertips will reach X_WF_target_ForDiffIK_, and
  /// send the command for the joint positions through LCM
  void UpdateIKTarget() {
    // update context of the plant
    plant_->tree()
        .get_mutable_multibody_state_vector(plant_context_.get())
        .head(16) = q_command_saved_;

    // set parameters for differential IK
    std::unique_ptr<DifferentialInverseKinematicsParameters> params_ =
        std::make_unique<DifferentialInverseKinematicsParameters>(16, 16);
    params_->set_nominal_joint_position(q_command_saved_);
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

    Eigen::Vector3d p_WTipcenter;
    bool target_updated_flag = false;
    Isometry3<double> p_FTipcenter;
    p_FTipcenter.setIdentity();
    p_FTipcenter.translation() += Eigen::Vector3d(0, 0, 0.012);
    for (int cur_finger = 0; cur_finger < 4; cur_finger++) {
      // if the new position is similar to the saved one, don't need to update
      Isometry3<double> X_W_TipCenter =
          X_WO_ * X_WF_target_ForDiffIK_[cur_finger] * p_FTipcenter;
      if (X_W_TipCenter_saved_[cur_finger].isApprox(X_W_TipCenter)) continue;

      // offset from the fingertip to the frame of the end link of the finger
      Eigen::Vector3d p_Finger_TipCenter(0, 0, 0.0267);
      const Frame<double>* FingertipFrame{nullptr};
      if (cur_finger == 0) {  // if it's the thumb
        p_Finger_TipCenter(2) = 0.0423;
        FingertipFrame = &(plant_->GetFrameByName("link_15"));
      } else {
        FingertipFrame = &(plant_->GetFrameByName(
            "link_" + std::to_string(cur_finger * 4 - 1)));
      }
      Vector6<double> V_WE_desired =
          manipulation::planner::ComputePoseDiffInCommonFrame(
              X_W_TipCenter_saved_[cur_finger], X_W_TipCenter);
      MatrixX<double> J_WE(6, 16);
      plant_->tree().CalcFrameGeometricJacobianExpressedInWorld(
          *plant_context_, *FingertipFrame, Vector3<double>::Zero(), &J_WE);

      DifferentialInverseKinematicsResult mbt_result =
          DoDifferentialInverseKinematics(q_command_saved_,
                                          Eigen::VectorXd::Zero(16),
                                          V_WE_desired, J_WE, *params_);
      if (mbt_result.status ==
          DifferentialInverseKinematicsStatus::kSolutionFound) {
        q_command_saved_ += mbt_result.joint_velocities.value();
        X_W_TipCenter_saved_[cur_finger] = X_W_TipCenter;
        target_updated_flag = true;
      }
    }
    if (target_updated_flag) SendJointCommand();
  }

  void SendJointCommand() {
    Eigen::VectorXd::Map(&allegro_command_.joint_position[0],
                         kAllegroNumJoints) = Px_position * q_command_saved_;
    lcm_.publish(kLcmCommandChannel, &allegro_command_);
  }

  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_allegro_status* status) {
    allegro_status_ = *status;
    hand_state.Update(allegro_status_);
    flag_hand_is_moving_ = !hand_state.IsAllFingersStuck();
  }

  void TargetFrameInput(const ::lcm::ReceiveBuffer*, const std::string&,
                        const robotlocomotion::pose_t* msg_X_WO_) {
    // update the pose of the mug
    X_WO_.setIdentity();
    X_WO_.translation() << msg_X_WO_->position.x, msg_X_WO_->position.y,
        msg_X_WO_->position.z;
    X_WO_.rotate(Eigen::Quaternion<double>(
        msg_X_WO_->orientation.w, msg_X_WO_->orientation.x,
        msg_X_WO_->orientation.y, msg_X_WO_->orientation.z));

    // display the target points for on the mug
    mug_state_.PublishTargetFingerPoseToLcm(X_WO_);

    if (updating_target_frame_) UpdateIKTarget();
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
  MatrixX<double> Px_position;
  std::vector<Isometry3<double>> X_W_TipCenter_saved_;
  // The current joint position targets that has been sent through LCM
  Eigen::VectorXd q_command_saved_;

  // control part for the target finger positions
  MugStateSet mug_state_;
  Isometry3<double> X_WO_;
  std::vector<Isometry3<double>> X_WF_target_;
  std::vector<Isometry3<double>> X_WF_target_ForDiffIK_;
};

int do_main() {
  HandPositionCommander runner;
  runner.RunCloseHand();
  runner.RunTranslation(1);
  runner.RunRotate(5.0, 'X', 1);
  return 0;
}

}  // namespace
}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::allegro_hand::do_main(); }
