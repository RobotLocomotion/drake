#pragma once

#include <memory>

#include "drake/geometry/geometry_system.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace multibody {
namespace objects_falling {

/// This method makes a MultibodyPlant model of the Acrobot - a canonical
/// underactuated system as described in <a
/// href="http://underactuated.mit.edu/underactuated.html?chapter=3">Chapter 3
/// of Underactuated Robotics</a>.
///
/// @param[in] default_parameters
///   Default parameters of the model set at construction. These parameters
///   include masses, link lengths, rotational inertias, etc. Refer to the
///   documentation of AcrobotParameters for further details.
/// @param[out] geometry_system
///   If a GeometrySystem is provided with this argument, this factory method
///   will register the new multibody plant to be a source for that geometry
///   system and it will also register geometry for visualization.
///   If this argument is omitted, no geometry will be registered.
std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeObjectsFallingPlant(
    double radius, double mass,const Vector3<double>& gravity,
    int nballs, int ncylinders,
    geometry::GeometrySystem<double>* geometry_system = nullptr);

}  // namespace objects_falling
}  // namespace multibody
}  // namespace examples
}  // namespace drake
