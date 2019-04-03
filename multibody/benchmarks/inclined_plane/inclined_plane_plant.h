#pragma once

#include <memory>

#include "drake/common/drake_optional.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace inclined_plane {

/// @name Inclined plane with a rigid body.
/// These functions create an inclined plane A and possibly a uniform-density
/// rigid body B with user-defined geometry and surface friction properties and
/// adds them to an existing plant.
/// The inclined plane can be modeled as an infinite half-space or as a box with
/// user-defined dimensions.  The inclined plane's top surface is located so
/// that it passes through point Wo (the origin of world W).  Right-handed
/// orthonormal vectors Wx, Wy, Wz are fixed in the world W.  Similarly,
/// Ax, Ay, Az are fixed in the inclined plane A and Bx, By, Bz are fixed in
/// body B.  Wz is directed vertically upward (opposite Earth's gravity).  The
/// inclined plane A is oriented by initially setting Ax=Wx, Ay=Wy, Az=Wz, and
/// then subjecting A to a right-handed rotation about Ay=Wy.
///
/// Some of these functions also add a rigid body B (e.g., a block or sphere)
/// with user-defined mass.
/// @anchor inclined_plane_parameters
/// @param[in] gravity The magnitude of Earth's local gravitational acceleration
///   in m/s².  Earth's gravity is in the world's -z direction.
/// @param[in] inclined_plane_angle Inclined plane angle (slope) i.e., angle
///   from Wx to Ax with positive sense Wy (radians).
/// @param[in] is_inclined_plane_half_space If `true`, the inclined plane is an
///   infinite half-space whereas `false` means the inclined is a box.
/// @param[in] inclined_plane_dimensions When the inclined plane is modeled as a
///   box, its dimensions (lengths) in the Ax, Ay, Az directions (meters).
///   Note: To be valid data, these dimensions must be positive.
/// @param[in] coefficient_friction_inclined_plane Coulomb's coefficient of
///   friction for inclined plane A (sliding friction and static friction).
/// @param[in] coefficient_friction_bodyB Coulomb's coefficient of friction
///   data for body B's surfaces (sliding friction and static friction).
/// @param[in] massB The mass of sphere B (kilograms), which must be positive.
/// @param[out] plant Plant that will contain inclined plane A.
/// @throws std::exception if plant is nullptr or there is invalid data.
/// @pre plant must be registered with a scene graph.
//@{

/// This method creates an inclined plane A and adds it to an existing plant.
/// @see @ref inclined_plane_parameters "Description of parameters"
void AddInclinedPlaneAndGravityToPlant(
    double gravity, double inclined_plane_angle,
    const optional<Vector3<double>>& inclined_plane_dimensions,
    const CoulombFriction<double>& coefficient_friction_inclined_plane,
    MultibodyPlant<double>* plant);

/// This method creates a uniform-density block (body B) and an inclined plane A
/// and adds both A and B to an existing plant.
/// @param[in] block_dimensions Dimensions (lengths) of block in the Bx, By, Bz
///   directions (meters). To be valid data, these dimensions must be positive.
/// @param[in] is_block_with_4Spheres This flag is `true` if block B's contact
///   with inclined plane A is modeled using 4 identical massless spheres
///   welded to the block B's four "bottom" corners, whereas this flag is
///   `false`if block B's contact is modeled with a block (box).
/// @see @ref inclined_plane_parameters "Description of other parameters"
void AddInclinedPlaneWithBlockToPlant(
    double gravity, double inclined_plane_angle,
    const optional<Vector3<double>>& inclined_plane_dimensions,
    const CoulombFriction<double>& coefficient_friction_inclined_plane,
    const CoulombFriction<double>& coefficient_friction_bodyB,
    double massB, const Vector3<double>& block_dimensions,
    bool is_block_with_4Spheres, MultibodyPlant<double>* plant);

/// This method creates a uniform-density sphere (body B) and an inclined plane
/// A and adds both A and B to an existing plant.
/// @param[in] radiusB The radius of sphere B (meters), which must be positive.
/// @see @ref inclined_plane_parameters "Description of other parameters"
/// @note Decorative visual geometry is added to the sphere to facilitate
/// visualizing the sphere's rotation.
void AddInclinedPlaneWithSphereToPlant(
    double gravity, double inclined_plane_angle,
    const optional<Vector3<double>>& inclined_plane_dimensions,
    const CoulombFriction<double>& coefficient_friction_inclined_plane,
    const CoulombFriction<double>& coefficient_friction_bodyB,
    double massB, double radiusB, MultibodyPlant<double>* plant);
//@}

}  // namespace inclined_plane
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
