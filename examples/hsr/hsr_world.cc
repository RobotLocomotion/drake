#include "drake/examples/hsr/hsr_world.h"

namespace drake {
namespace examples {
namespace hsr {

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;

template <typename T>
HsrWorld<T>::HsrWorld(const std::string& config_file)
    : config_file_(config_file),
      owned_plant_(std::make_unique<MultibodyPlant<T>>(1e-3)),
      owned_scene_graph_(std::make_unique<SceneGraph<T>>()) {
  // This class holds the unique_ptrs explicitly for plant and scene_graph
  // until Finalize() is called (when they are moved into the Diagram). Grab
  // the raw pointers, which should stay valid for the lifetime of the Diagram.
  scene_graph_ = owned_scene_graph_.get();
  scene_graph_->set_name("scene_graph");

  plant_ = owned_plant_.get();
  plant_->RegisterAsSourceForSceneGraph(scene_graph_);
  plant_->set_name("plant");

  this->set_name("hsr_world");

  // Load the models here. Something like
  // const auto config_params = LoadWorldConfigurationParameters(config_file_);

  // Parse urdfs to get the models from the configuration parameters.
  // SetupWorld(config_params);

  // This function will finalize the plant and all the ports.
  // Finalize();
}

}  // namespace hsr
}  // namespace examples
}  // namespace drake