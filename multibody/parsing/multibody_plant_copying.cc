#include "drake/multibody/parsing/multibody_plant_copying.h"

#include "drake/multibody/parsing/detail_multibody_plant_subgraph.h"

namespace drake {
namespace multibody {

ModelInstanceIndex CloneModelInstanceTo(
    const MultibodyPlant<double>& src_plant, MultibodyPlant<double>* dest_plant,
    ModelInstanceIndex model_instance) {
  auto elems = internal::MultibodyPlantElements::FromPlant(&src_plant);
  internal::MultibodyPlantSubgraph copy_subgraph(elems);
  for (auto model : elems.model_instances()) {
    if (model != model_instance) {
      copy_subgraph.RemoveModelInstance(model);
    }
  }
  auto elements_map = copy_subgraph.AddTo(dest_plant);
  return elements_map.model_instances().at(model_instance);
}
}  // namespace multibody
}  // namespace drake
