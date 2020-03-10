#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>

#include "drake/common/find_resource.h"
#include "drake/lcmt_hsr_sim_command.hpp"
#include "drake/lcmt_hsr_sim_status.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/joint_actuator.h"

namespace drake {
namespace examples {
namespace hsr {

/// A utility class to prepare a common configuration to test lcm related
/// functionalities.

class SimLcmTestConfig final {
 public:
  explicit SimLcmTestConfig(const std::string& robot_urdf_path) {
    const std::string model_path = FindResourceOrThrow(robot_urdf_path);
    multibody::Parser(&robot_plant_).AddModelFromFile(model_path);
    robot_plant_.set_name("robot_plant");
    robot_plant_.Finalize();

    SetupEstimatedAndDesiredStateVectors();

    sim_status_message_ = CreateSimStatusMessage();
    sim_command_message_ = CreateSimCommandMessage();
  }

  const multibody::MultibodyPlant<double>& robot_plant() const {
    return robot_plant_;
  }

  const lcmt_hsr_sim_status& sim_status_message() const {
    return sim_status_message_;
  }

  const lcmt_hsr_sim_command& sim_command_message() const {
    return sim_command_message_;
  }

  const Eigen::VectorXd& desired_state_vector() const {
    return desired_state_vector_;
  }

  const Eigen::VectorXd& estimated_state_vector() const {
    return estimated_state_vector_;
  }

  double CalcEstimatedStateValue(int actuator_index) const {
    return static_cast<double>(actuator_index * actuator_index);
  }

  double CalcDesiredStateValue(int actuator_index) const {
    return static_cast<double>(actuator_index * actuator_index +
                               actuator_index / 10.0);
  }

 private:
  const lcmt_hsr_sim_status CreateSimStatusMessage() const {
    lcmt_hsr_sim_status message;
    message.utime = 1;
    message.num_joints = robot_plant_.num_actuators();
    for (multibody::JointActuatorIndex i(0); i < robot_plant_.num_actuators();
         ++i) {
      const auto& joint_i = robot_plant_.get_joint_actuator(i).joint();
      message.joint_name.push_back(joint_i.name());
      message.joint_position.push_back(CalcEstimatedStateValue(i));
      message.joint_velocity.push_back(CalcEstimatedStateValue(i));
      message.joint_torque.push_back(0.0);
    }
    return message;
  }

  const lcmt_hsr_sim_command CreateSimCommandMessage() const {
    lcmt_hsr_sim_command message;
    message.utime = 1;
    message.num_joints = robot_plant_.num_actuators();
    for (multibody::JointActuatorIndex i(0); i < robot_plant_.num_actuators();
         ++i) {
      const auto& joint_i = robot_plant_.get_joint_actuator(i).joint();
      message.joint_name.push_back(joint_i.name());
      message.joint_position.push_back(CalcDesiredStateValue(i));
      message.joint_velocity.push_back(CalcDesiredStateValue(i));
    }
    return message;
  }

  void SetupEstimatedAndDesiredStateVectors() {
    const int state_size =
        robot_plant_.num_positions() + robot_plant_.num_velocities();
    estimated_state_vector_ = Eigen::VectorXd(state_size);
    desired_state_vector_ = Eigen::VectorXd(state_size);
    // Set the w element of the quaternion representation to 1.
    estimated_state_vector_[0] = 1.0;
    desired_state_vector_[0] = 1.0;

    for (int i = 4; i < state_size; ++i) {
      estimated_state_vector_[i] = CalcEstimatedStateValue(i);
      desired_state_vector_[i] = CalcDesiredStateValue(i);
    }
  }

  multibody::MultibodyPlant<double> robot_plant_{0.001};
  Eigen::VectorXd desired_state_vector_;
  Eigen::VectorXd estimated_state_vector_;

  lcmt_hsr_sim_command sim_command_message_;
  lcmt_hsr_sim_status sim_status_message_;
};

}  // namespace hsr
}  // namespace examples
}  // namespace drake
