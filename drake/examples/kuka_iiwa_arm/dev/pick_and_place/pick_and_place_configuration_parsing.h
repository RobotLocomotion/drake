#pragma once

#include <string>
#include <vector>

#include "drake/common/type_safe_index.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

using TaskIndex = TypeSafeIndex<class TaskTag>;

pick_and_place::PlannerConfiguration ParsePlannerConfigurationOrThrow(
    const std::string& filename, TaskIndex task_index = TaskIndex(0));

pick_and_place::PlannerConfiguration ParsePlannerConfigurationOrThrow(
    const std::string& filename,
    const std::string& end_effector_name,
    pick_and_place::RobotBaseIndex robot_base_index =
        pick_and_place::RobotBaseIndex(0),
    pick_and_place::TargetIndex target_index = pick_and_place::TargetIndex(0));

std::vector<pick_and_place::PlannerConfiguration>
ParsePlannerConfigurationsOrThrow(const std::string& filename);

pick_and_place::SimulatedPlantConfiguration
ParseSimulatedPlantConfigurationOrThrow(const std::string& filename);

pick_and_place::SimulatedPlantConfiguration
ParseSimulatedPlantConfigurationStringOrThrow(const std::string& configuration);

pick_and_place::OptitrackConfiguration ParseOptitrackConfigurationOrThrow(
    const std::string& filename);

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
