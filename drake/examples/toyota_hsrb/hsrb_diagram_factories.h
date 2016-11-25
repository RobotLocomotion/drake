#pragma once

#include <memory>
#include <string>
#include <tuple>

#include "drake/lcm/drake_lcm_interface.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace toyota_hsrb {

// TODO(liang.fok) The following two methods are likely of general use. Once the
// rule-of-three is satisfied, they should be moved to a more general location
// (probably somewhere in drake-distro/drake/multibody/).

/**
 * Creates and returns a systems::Diagram containing a systems::RigidBodyPlant
 * and a systems::DrakeVisualizer. The plant's output port zero is connected to
 * the systems::DrakeVisualizer's input port zero. The returned
 * systems::Diagram has one input port that's connected to the plant's input
 * port zero (which accepts the effort command), and one output port that's
 * connected to the plant's output port zero (which contains the plant's
 * generalized state).
 */
std::unique_ptr<systems::Diagram<double>> CreatePlantAndVisualizerDiagram(
    const std::string& urdf_string, double penetration_stiffness,
    double penetration_damping, double friction_coefficient,
    lcm::DrakeLcmInterface* lcm, systems::RigidBodyPlant<double>** plant);

/**
 * Connects a constant source to the input port of @p plant_diagram and returns
 * the resulting systems::Diagram. The returned systems::Diagram has no input
 * ports and one output port that's wired to the output port zero of
 * @p plant_diagram (which contains the plant's generalized state).
 */
std::unique_ptr<systems::Diagram<double>> CreateConstantSourceToPlantDiagram(
    const systems::RigidBodyPlant<double>& plant,
    std::unique_ptr<systems::Diagram<double>> plant_diagram);

}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake
