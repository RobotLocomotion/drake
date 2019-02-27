#pragma once

#include <memory>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace block_on_inclined_plane {

/// This method creates geometry for a block, an inclined plane, and horizontal
/// plane (World).  It creates the surface friction properties of the block and
/// inclined plane and the mass/inertia properties of a uniform-density block.
///
/// @param[in] Lx The length of the block in the block's x-direction (meters).
/// @param[in] Ly The length of the block in the block's y-direction (meters).
/// @param[in] Lz The length of the block in the block's z-direction (meters).
/// @param[in] mass The block's mass (kilograms).
/// @param[in] slope The inclined plane's slope (radians).
/// @param[in] gravity The magnitude of Earth's local gravitational acceleration
///   in m/s².  Earth's gravity is in the World's -z direction.
/// @param[in] coefficient_of_friction_block Coulomb's coefficient of friction
///   data for the block (sliding friction, static friction, .
/// @param[in] coefficient_of_friction_inclined_plane Coulomb's coefficient of
///   friction for the inclined plane for either sliding or static friction.
/// @param[out] plant Plant to contain the block and inclined plane.
/// @throws std::exception if plant is nullptr.
/// @pre plant must be registered with a scene graph.
void AddBlockAndInclinedPlaneToPlant(
    double Lx, double Ly, double Lz, double mass, double slope, double gravity,
    const CoulombFriction<double>& coefficient_of_friction_block,
    const CoulombFriction<double>& coefficient_of_friction_inclined_plan,
    MultibodyPlant<double>* plant);

}  // namespace block_on_inclined_plane
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
