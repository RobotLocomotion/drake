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

/// Read the pick and place configuration from @p filename, returning
/// the planner configuration for the task specified by @p task_index.
pick_and_place::PlannerConfiguration ParsePlannerConfigurationOrThrow(
    const std::string& filename, TaskIndex task_index = TaskIndex(0));

/// Read the pick and place configuration from @p filename, returning
/// the planner configuration for all tasks.
std::vector<pick_and_place::PlannerConfiguration>
ParsePlannerConfigurationsOrThrow(const std::string& filename);

/// Read the pick and place configuration from @p filename, returning
/// the simulated plant configuration.
pick_and_place::SimulatedPlantConfiguration
ParseSimulatedPlantConfigurationOrThrow(const std::string& filename);

/// Parse the pick and place configuration from @p configuation,
/// returning the simulated plant configuration.
pick_and_place::SimulatedPlantConfiguration
ParseSimulatedPlantConfigurationStringOrThrow(const std::string& configuration);


/// Read the pick and place configuration from @p filename, returning
/// the optitrack information for the scenario.
pick_and_place::OptitrackConfiguration ParseOptitrackConfigurationOrThrow(
    const std::string& filename);

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
