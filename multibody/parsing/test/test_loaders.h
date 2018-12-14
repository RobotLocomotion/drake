#pragma once

#include <functional>
#include <string>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace test {

/// This is a function signature used to parameterize unit tests by which
/// loading mechanism they use.
typedef std::function<void(
    const std::string& base_name,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph)> ModelLoadFunction;

/// Load the model from an SDF with a resource path stem of @p base_name into
/// @p plant.
void LoadFromSdf(
    const std::string& base_name,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph);

/// Load the model from an URDF with a resource path stem of @p base_name into
/// @p plant.
void LoadFromUrdf(
    const std::string& base_name,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph);

}  // namespace test
}  // namespace multibody
}  // namespace drake
