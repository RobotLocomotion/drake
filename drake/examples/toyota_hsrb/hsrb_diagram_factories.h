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

// TODO(liang.fok) Consider an alternative design of having the following two
// methods operate on a DiagramBuilder object, which will allow the creation of
// of an un-nested Diagram.

/**
 * Builds and returns a systems::Diagram containing a systems::RigidBodyPlant
 * and a systems::DrakeVisualizer. The plant's output port zero is connected to
 * the systems::DrakeVisualizer's input port zero. The returned
 * systems::Diagram has the same input and output ports as the plant.
 */
std::unique_ptr<systems::Diagram<double>> BuildPlantAndVisualizerDiagram(
    const std::string& urdf_string, double penetration_stiffness,
    double penetration_damping, double friction_coefficient,
    lcm::DrakeLcmInterface* lcm, systems::RigidBodyPlant<double>** plant);

/**
 * Builds and returns a systems::Diagram consisting of a
 * systems::ConstantVectorSource connected to input port zero of
 * @p plant_diagram. The returned systems::Diagram has no input ports and the
 * same output ports as @p plant_diagram. Typically, @p plant_diagram is built
 * using BuildPlantAndVisualizerDiagram().
 */
std::unique_ptr<systems::Diagram<double>> BuildConstantSourceToPlantDiagram(
    std::unique_ptr<systems::Diagram<double>> plant_diagram);

}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake
