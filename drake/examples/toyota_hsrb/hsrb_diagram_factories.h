#pragma once

#include <memory>
#include <string>
#include <tuple>

#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace toyota_hsrb {

/**
 * Creates and returns a systems::Diagram containing a systems::RigidBodyPlant
 * with a model of Toyota's HSRb robot that's connected to a DrakeVisualizer.
 * It has one input port that's connected to the plant's input port zero, and
 * one output port that's connected to the plant's output port zero.
 */
std::unique_ptr<systems::Diagram<double>> CreateHsrbPlantDiagram(
    const std::string& hsrb_urdf_string, double penetration_stiffness,
    double penetration_damping, double friction_coefficient,
    lcm::DrakeLcm* lcm, systems::RigidBodyPlant<double>** plant);

/**
 * Adds a constant source input to @p plant_diagram and returns the resulting
 * diagram. The returned Diagram has no input ports and one output port that's
 * wired to the output port zero of @p plant_diagram.
 */
std::unique_ptr<systems::Diagram<double>>
CreateHsrbDemo1Diagram(const systems::RigidBodyPlant<double>& plant,
	                   std::unique_ptr<systems::Diagram<double>> plant_diagram);

}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake
