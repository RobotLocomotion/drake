#pragma once

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {

// Copies a single model instance from one plant to another
//
// @throws std::exception if the model failed to be copied into the destination
// plant.
// @param src_plant
//    The source plant
// @param dest_plant
//    The destination plant
// @param model_instances
//    The model instane from the source plant to be copied.
//  @returns The model instance index for the copied model in the destination
//  plant.
ModelInstanceIndex CloneModelInstanceTo(
    const MultibodyPlant<double>& src_plant, MultibodyPlant<double>* dest_plant,
    ModelInstanceIndex model_instances);

}  // namespace multibody
}  // namespace drake
