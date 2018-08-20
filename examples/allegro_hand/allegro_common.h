#pragma once

#include <string>
#include <vector>
#include <map>

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace allegro_hand {

constexpr int kAllegroNumJoints = 16;

/// Used to set the feedback gains for the simulated position control
void SetPositionControlledGains(Eigen::VectorXd* Kp,
                                    Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd);

void GetControlPortMapping(
    multibody::multibody_plant::MultibodyPlant<double>& plant, 
    MatrixX<double>& Px, MatrixX<double>& Py);

const Eigen::VectorXd SetTargetJointPose();
const std::map<std::string, int> SetJointNameMapping();


}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake