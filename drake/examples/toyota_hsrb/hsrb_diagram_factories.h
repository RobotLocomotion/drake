#pragma once

#include <memory>
#include <string>

#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace toyota_hsrb {

/**
 * Defines the name of the RigidBodyPlant system.
 */
// static const char* const kPlantName;

/**
 * Obtains the RigidBodyPlant from within the provided diagram. The provided
 * diagram must contain a subsystem that is named "RigidBodyPlant".
 */
// const systems::RigidBodyPlant<double>* GetRigidBodyPlant(
//     const systems::Diagram<double>* diagram);

/**
 * Creates and returns a 2-tuple consisting of (1) an unique pointer to a
 * systems::Diagram and (2) a pointer to the diagram's systems::RigidBodyPlant.
 *
 * The returned systems::Diagram contains the plant connected to a
 * DrakeVisualizer. It contains one input port that's connected to the plant's
 * input port zero, and one output port that's connected to the plant's output
 * port zero.
 *
 * The systems::RigidBodyPlant contains a systems::RigidBodyTree modeling
 * Toyota's HSRb robot.
 */
std::tuple<std::unique_ptr<Diagram<double>>, RigidBodyPlant*>
CreateHsrPlantDiagram(const std::string& urdf_string,
    double penetration_stiffness,
    double penetration_damping,
    double friction_coefficient,
    lcm::DrakeLcm* lcm);

/**
 * Wraps the provided diagram with a constant source input. The returned Diagram
 * has no input ports and one output port that's wired to the output port of
 * @p plant_diagram.
 */
std::unique_ptr<Diagram<double>
CreateHsrDemo1Diagram(std::unique_ptr<Diagram<double> plant_diagram);

}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake
