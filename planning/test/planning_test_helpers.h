#pragma once

#include <memory>
#include <string>

#include "drake/multibody/parsing/model_directives.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/robot_diagram.h"

namespace drake {
namespace planning {
namespace test {

/* Creates a RobotDiagram from the given directives. */
std::unique_ptr<RobotDiagram<double>> MakePlanningTestModel(
    const multibody::parsing::ModelDirectives& directives);

/* Creates a RobotDiagram from the given model data string.
The model_ext is the file format extension (e.g., "dmd.yaml").
The model_data is the model file contents. */
std::unique_ptr<RobotDiagram<double>> MakePlanningTestModel(
    const std::string& model_ext, const std::string& model_contents);

/* Returns a vector of non-uniform distance weights for a 7-dof iiwa. */
Eigen::VectorXd GetIiwaDistanceWeights();

/* Returns a particular ConfigurationDistanceFunction for a 7-dof iiwa.
The weights are non-uniform. */
ConfigurationDistanceFunction MakeWeightedIiwaConfigurationDistanceFunction();

// TODO(rpoyner-tri): fix the naming scheme, maybe.
/* Adds a new model to @p plant, consisting of @p n bodies, connected by
revolute joints. The geometry for each body consists of @p num_geo small
spheres.  @note: the model name is based on @p n, so adding chains of the
same length will fail. */
multibody::ModelInstanceIndex AddChain(multibody::MultibodyPlant<double>* plant,
                                       int n, int num_geo = 1);

}  // namespace test
}  // namespace planning
}  // namespace drake
