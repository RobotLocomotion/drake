#pragma once

#include <memory>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace block_4Spheres_on_inclined_plane {

/// This method creates geometry and surface friction properties for a
/// uniform-density block B having four identical massless spheres welded to
/// its four "bottom" corners.  It also creates geometry and surface friction
/// properties for an inclined-plane (half-space) A.  The bottom surface of
/// block B makes contact with the top surface of inclined-plane A.
/// Right-handed sets of orthonormal vectors Wx, Wy, Wz; Ax, Ay, Az; Bx, By, Bz;
/// are fixed in the World W, inclined-plane A, and block B, respectively.
/// Wz is directed vertically upward (opposite Earth's gravity).
/// The inclined-plane is oriented by initially setting Ax=Wx, Ay=Wy, Az=Wz, and
/// then subjecting A to a right-handed rotation in W about Ay=Wy, so that Ax
/// is directed downhill.
///
/// @param[in] Lx The length of the block in the block's x-direction (meters).
/// @param[in] Ly The length of the block in the block's y-direction (meters).
/// @param[in] Lz The length of the block in the block's z-direction (meters).
/// @param[in] mass The block's mass (kilograms).
/// @param[in] slope The inclined plane's slope (radians).
/// @param[in] gravity The magnitude of Earth's local gravitational acceleration
///   in m/s².  Earth's gravity is in the World's -z direction.
/// @param[in] coefficient_of_friction_block Coulomb's coefficient of friction
///   data for the block and its spheres (sliding friction, static friction).
/// @param[in] coefficient_of_friction_inclined_plane Coulomb's coefficient of
///   friction for the inclined plane (sliding friction, static friction).
/// @param[out] plant Plant that contains the block and inclined plane.
/// @throws std::exception if plant is nullptr.
/// @pre plant must be registered with a scene graph.
void AddBlockWith4SpheresAndInclinedPlaneToPlant(
    double Lx, double Ly, double Lz, double mass, double slope, double gravity,
    const CoulombFriction<double>& coefficient_of_friction_block,
    const CoulombFriction<double>& coefficient_of_friction_inclined_plan,
    MultibodyPlant<double>* plant);

}  // namespace block_4Spheres_on_inclined_plane
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
