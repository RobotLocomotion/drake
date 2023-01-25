#pragma once

#include <memory>

#include "drake/multibody/parsing/model_directives.h"
#include "drake/planning/collision_checker.h"
#include "planning/robot_diagram.h"

namespace anzu {
namespace planning {
std::unique_ptr<RobotDiagram<double>> MakePlanningTestModel(
    const drake::multibody::parsing::ModelDirectives& directives);

drake::planning::ConfigurationDistanceFunction
MakeWeightedIiwaConfigurationDistanceFunction();

// TODO(rpoyner-tri): fix the naming scheme, maybe.
/* Adds a new model to @p plant, consisting of @p n bodies, connected by
   revolute joints. The geometry for each body consists of @p num_geo small
   spheres.  @note: the model name is based on @p n, so adding chains of the
   same length will fail. */
drake::multibody::ModelInstanceIndex
AddChain(drake::multibody::MultibodyPlant<double>* plant, int n,
         int num_geo = 1);

}  // namespace planning
}  // namespace anzu
