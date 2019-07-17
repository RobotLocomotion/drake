/// @file
///
/// Demo of moving the jaco's end effector in cartesian space.  This program
/// uses differential inverse kinematics to move the end effector from the
/// current position to the location specified on the command line.  The
/// current calculated position of the end effector is printed before, during,
/// and after the commanded motion.

#include <gflags/gflags.h>
#include "lcm/lcm-cpp.hpp"

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcmt_jaco_command.hpp"
#include "drake/lcmt_jaco_status.hpp"
#include "drake/manipulation/planner/differential_inverse_kinematics.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_string(lcm_status_channel, "KINOVA_JACO_STATUS",
              "Channel on which to listen for lcmt_jaco_status messages.");
DEFINE_string(lcm_command_channel, "KINOVA_JACO_COMMAND",
              "Channel on which to send for lcmt_jaco_command messages.");
DEFINE_double(x, 0.3, "x coordinate (meters) to move to");
DEFINE_double(y, -0.26, "y coordinate (meters) to move to");
DEFINE_double(z, 0.5, "z coordinate (meters) to move to");
DEFINE_double(roll, -1.7,
              "target roll (radians) about world x axis for end effector");
DEFINE_double(pitch, -1.3,
              "target pitch (radians) about world y axis for end effector");
DEFINE_double(yaw, -1.8,
              "target yaw (radians) about world z axis for end effector");
DEFINE_string(ee_name, "j2s7s300_end_effector",
              "Name of the end effector link");

using drake::manipulation::planner::DifferentialInverseKinematicsResult;
using drake::manipulation::planner::DifferentialInverseKinematicsStatus;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

namespace drake {
namespace examples {
namespace kinova_jaco_arm {
namespace {

const char kUrdfPath[] =
    "drake/manipulation/models/jaco_description/urdf/j2s7s300.urdf";

class MoveDemoRunner {
 public:
  MoveDemoRunner() {
    urdf_ =
        (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kUrdfPath));
    instance_ = Parser(&plant_).AddModelFromFile(urdf_);
    plant_.WeldFrames(plant_.world_frame(), plant_.GetFrameByName("base"));
    plant_.Finalize();
    context_ = plant_.CreateDefaultContext();

    lcm_.subscribe(FLAGS_lcm_status_channel,
                   &MoveDemoRunner::HandleStatus, this);

    params_ = manipulation::planner::DifferentialInverseKinematicsParameters(
        plant_.num_positions(), plant_.num_velocities());
    params_.set_joint_position_limits(std::make_pair(
        plant_.GetPositionLowerLimits(), plant_.GetPositionUpperLimits()));
    params_.set_joint_velocity_limits(std::make_pair(
        plant_.GetVelocityLowerLimits(), plant_.GetVelocityUpperLimits()));
    params_.set_joint_acceleration_limits(std::make_pair(
        plant_.GetAccelerationLowerLimits(),
        plant_.GetAccelerationUpperLimits()));
  }

  void Run() {
    while (lcm_.handle() >= 0) { }
  }

 private:
  // Handle the incoming status message from the jaco.  This is a
  // fairly high rate operation (100Hz is typical).  The plan is
  // calculated on the first status message received, after that
  // periodically display the current position of the end effector.
  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_jaco_status* status) {
    status_count_++;

    const int num_q = status->num_joints + status->num_fingers;

    Eigen::VectorXd jaco_q = Eigen::VectorXd::Zero(num_q);
    Eigen::VectorXd jaco_v = Eigen::VectorXd::Zero(num_q);

    // Don't bother with finger position, it's not in the right units for MBP
    // and we don't actually care.
    for (int i = 0; i < status->num_joints; i++) {
      jaco_q[i] = status->joint_position[i];
      jaco_v[i] = status->joint_velocity[i];
    }

    plant_.SetPositions(context_.get(), instance_, jaco_q);
    plant_.SetVelocities(context_.get(), instance_, jaco_v);

    // Only print the position every 100 messages (this results in an
    // 0.5s period in a typical configuration).
    if (status_count_ % 100 == 1) {
      // Estimate the end effector position through forward kinematics.
      const multibody::Body<double>& ee_body =
          plant_.GetBodyByName(FLAGS_ee_name);
      const math::RigidTransformd& ee_pose =
          plant_.EvalBodyPoseInWorld(*context_, ee_body);
      const math::RollPitchYawd rpy(ee_pose.rotation());
      drake::log()->info("End effector at: {} {}",
                         ee_pose.translation().transpose(),
                         rpy.vector().transpose());
    }

    // Once we've got enough status messages to calculate dt, start trying to
    // move.
    if (status_count_ >= 2) {
      const multibody::Body<double>& ee_body =
          plant_.GetBodyByName(FLAGS_ee_name);

      math::RigidTransformd X_WE_desired(
          math::RollPitchYawd(FLAGS_roll, FLAGS_pitch, FLAGS_yaw),
          Eigen::Vector3d(FLAGS_x, FLAGS_y, FLAGS_z));

      DifferentialInverseKinematicsResult result =
          manipulation::planner::DoDifferentialInverseKinematics(
              plant_, *context_, X_WE_desired, ee_body.body_frame(), params_);
      if (result.status ==
          DifferentialInverseKinematicsStatus::kSolutionFound) {
      } else {
        drake::log()->error("IK solve failed: {}", result.status);
        exit(1);
      }

      lcmt_jaco_command command{};
      command.utime = status->utime;
      command.num_joints = status->num_joints;
      command.joint_position.resize(status->num_joints);
      command.joint_velocity.resize(status->num_joints);

      const double dt = (status->utime / 1e6) - (last_status_utime_ / 1e6);

      for (int i = 0; i < status->num_joints; ++i) {
        command.joint_position[i] = status->joint_position[i] +
            (*result.joint_velocities)[i] * dt;
        command.joint_velocity[i] = (*result.joint_velocities)[i];
      }

      command.num_fingers = status->num_fingers;
      command.finger_position.resize(status->num_fingers);
      command.finger_velocity.resize(status->num_fingers);

      for (int i = 0; i < status->num_fingers; ++i) {
        command.finger_position[i] = status->finger_position[i];
        command.finger_velocity[i] = 0;
      }

      lcm_.publish(FLAGS_lcm_command_channel, &command);
    }

    last_status_utime_ = status->utime;
  }

  ::lcm::LCM lcm_;
  std::string urdf_;
  MultibodyPlant<double> plant_;
  multibody::ModelInstanceIndex instance_;
  std::unique_ptr<systems::Context<double>> context_;
  int status_count_{0};
  int64_t last_status_utime_{0};
  manipulation::planner::DifferentialInverseKinematicsParameters params_;
};

}  // namespace
}  // namespace kinova_jaco_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  drake::examples::kinova_jaco_arm::MoveDemoRunner runner;
  runner.Run();
}
