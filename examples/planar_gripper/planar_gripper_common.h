#pragma once

#include <string>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace planar_gripper {

using drake::multibody::MultibodyPlant;

template <typename T>
void WeldGripperFrames(MultibodyPlant<T>* plant);

void PublishRobotPlan(const robotlocomotion::robot_plan_t& plan);

MatrixX<double> ParseKeyframes(const std::string& name,
                               EigenPtr<Vector3<double>> brick_ics);

}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
