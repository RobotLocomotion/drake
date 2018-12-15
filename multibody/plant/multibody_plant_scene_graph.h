/** @file
Provides sugar methods to construct a MultibodyPlant and SceneGraph, build
them into the same diagram.
@warning This will possibly disappear pending resolution of #9691.
*/

#pragma once

#include <memory>
#include <utility>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {

/// Adds a MultibodyPlant and a SceneGraph instance to a diagram builder,
/// connecting the geometry ports.
/// @param[out] builder
///   Builder to add to.
/// @param[in] plant (optional)
///   Constructed plant (e.g. for using a discrete plant).
/// @param[in] scene_graph (optional)
///   Constructed scene graph.
/// @return Pair of the registered plant and scene graph.
template <typename T>
std::pair<MultibodyPlant<T>*, geometry::SceneGraph<T>*>
AddMultibodyPlantSceneGraph(
    systems::DiagramBuilder<T>* builder,
    std::unique_ptr<MultibodyPlant<T>> plant = nullptr,
    std::unique_ptr<geometry::SceneGraph<T>> scene_graph = nullptr) {
  DRAKE_DEMAND(builder != nullptr);
  if (!plant) {
    plant = std::make_unique<MultibodyPlant<T>>();
  }
  if (!scene_graph) {
    scene_graph = std::make_unique<geometry::SceneGraph<T>>();
  }
  auto* plant_ptr = builder->AddSystem(std::move(plant));
  plant_ptr->set_name("plant");
  auto* scene_graph_ptr = builder->AddSystem(std::move(scene_graph));
  scene_graph_ptr->set_name("scene_graph");
  plant_ptr->RegisterAsSourceForSceneGraph(scene_graph_ptr);
  builder->Connect(
      plant_ptr->get_geometry_poses_output_port(),
      scene_graph_ptr->get_source_pose_port(
          plant_ptr->get_source_id().value()));
  builder->Connect(
      scene_graph_ptr->get_query_output_port(),
      plant_ptr->get_geometry_query_input_port());
  return {plant_ptr, scene_graph_ptr};
}

}  // namespace multibody
}  // namespace drake
