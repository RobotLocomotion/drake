#pragma once

#include <memory>
#include <vector>

#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/output_port.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

/// Build a RigidBodyPlant for a particular pick and place
/// configuration.  If @p wsg_instances is nullptr, no grippers will
/// be added to the simulation (this could be useful if, for example,
/// your arm model includes a gripper).
std::unique_ptr<systems::RigidBodyPlant<double>> BuildPickAndPlacePlant(
    const pick_and_place::SimulatedPlantConfiguration& configuration,
    std::vector<manipulation::util::ModelInstanceInfo<double>>* arm_instances,
    std::vector<manipulation::util::ModelInstanceInfo<double>>* wsg_instances,
    std::vector<manipulation::util::ModelInstanceInfo<double>>*
    object_instances,
    std::vector<manipulation::util::ModelInstanceInfo<double>>*
    table_instances);

/// Add the components needed for a simulated optitrack to a diagram.
/// @p tree and the model instance info would typically be created by
/// BuildPickAndPlacePlant.  @p kinematics_port is the kinematics
/// results output port from a RigidBodyPlant.
const systems::OutputPort<double>& AddOptitrackComponents(
    const pick_and_place::OptitrackConfiguration& optitrack_configuration,
    const RigidBodyTree<double>& tree,
    const std::vector<manipulation::util::ModelInstanceInfo<double>>&
    arm_instances,
    const std::vector<manipulation::util::ModelInstanceInfo<double>>&
    object_instances,
    const std::vector<manipulation::util::ModelInstanceInfo<double>>&
    table_instances,
    const systems::OutputPort<double>& kinematics_port,
    systems::DiagramBuilder<double>* builder);

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
