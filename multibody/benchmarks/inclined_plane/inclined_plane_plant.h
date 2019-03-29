#pragma once

#include <memory>

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace inclined_plane {

/// This method creates an inclined-plane with user-defined geometry and surface
/// friction properties and adds it to an existing plant.
/// The inclined-plane can be modeled as an infinite half-space or as a box with
/// user-defined dimensions.  The inclined-plane's top surface is located so
/// that it passes through point Wo (the origin of world W).  Right-handed sets
/// of orthonormal vectors Wx, Wy, Wz and Ax, Ay, Az are fixed in the world W
/// and inclined-plane A, respectively.  Wx is directed horizontally left and
/// Wz is directed vertically upward (opposite Earth's gravity).
/// Plane A is oriented by initially setting Ax=Wx, Ay=Wy, Az=Wz, and then
/// subjecting A to a right-handed rotation about Ay=Wy.
/// Note: For a small positive rotation angle, Ax points downhill (down/left).
/// @param[in] gravity The magnitude of Earth's local gravitational acceleration
///   in m/s².  Earth's gravity is in the world's -z direction.
/// @param[in] inclined_plane_angle Inclined-plane angle (slope) i.e., angle
///   from Wx to Ax with positive sense Wy (radians).
/// @param[in] is_inclined_plane_half_space If `true` the inclined-plane is an
///   infinite half-space whereas `false` means the inclined is a box.
/// @param[in] inclined_plane_dimensions When the inclined-plane is modeled as a
///   box, its dimensions (lengths) in the Ax, Ay, Az directions (meters).
///   Note: To be valid data, these dimensions must be positive.
/// @param[in] coefficient_friction_inclined_plane Coulomb's coefficient of
///   friction for inclined-plane A (sliding friction and static friction).
/// @param[out] plant Plant that will contain inclined-plane A.
/// @throws std::exception if plant is nullptr or there is invalid data.
/// @pre plant must be registered with a scene graph.
void AddInclinedPlaneAndGravityToPlant(
    double gravity, double inclined_plane_angle,
    bool is_inclined_plane_half_space,
    const Vector3<double>& inclined_plane_dimensions,
    const CoulombFriction<double>& coefficient_friction_inclined_plane,
    MultibodyPlant<double>* plant);

/// This method creates an inclined-plane A and uniform-density block (body B)
/// with user-defined mass, geometry, and surface friction properties and adds
/// them to an existing plant.  B's bottom surface contacts A's top surface.
/// Right-handed sets of orthonormal vectors Wx, Wy, Wz; Ax, Ay, Az; Bx, By, Bz;
/// are fixed in the world W, inclined-plane A, and block B, respectively.
/// The inclined-plane is discussed in AddInclinedPlaneAndGravityToPlant().
/// @param[in] gravity See AddInclinedPlaneAndGravityToPlant()
/// @param[in] inclined_plane_angle See AddInclinedPlaneAndGravityToPlant()
/// @param[in] is_inclined_plane_half_space See
/// AddInclinedPlaneAndGravityToPlant()
/// @param[in] inclined_plane_dimensions See
/// AddInclinedPlaneAndGravityToPlant()
/// @param[in] block_dimensions Dimensions (lengths) of block in the Bx, By, Bz
///   directions (meters). To be valid data, these dimensions must be positive.
/// @param[in] massB The mass of block B (kilograms), which must be positive.
/// @param[in] coefficient_friction_inclined_plane Coulomb's coefficient of
///   friction for inclined-plane A (sliding friction and static friction).
/// @param[in] coefficient_friction_bodyB Coulomb's coefficient of friction
///   data for body B's surfaces (sliding friction and static friction).
/// @param[in] is_block_with_4Spheres This flag is `true` if block B's contact
///   with inclined-plane A is modeled using 4 identical massless spheres
///   welded to the block B's four "bottom" corners, whereas this flag is
///   `false`if block B's contact is modeled with a block (box).
/// @param[out] plant Plant that  will contain inclined-plane A and body B.
/// @throws std::exception if plant is nullptr or there is invalid data.
/// @pre plant must be registered with a scene graph.
void AddInclinedPlaneWithBlockToPlant(
    double gravity, double inclined_plane_angle,
    bool is_inclined_plane_half_space,
    const Vector3<double>& inclined_plane_dimensions,
    const Vector3<double>& block_dimensions, double massB,
    const CoulombFriction<double>& coefficient_friction_inclined_plane,
    const CoulombFriction<double>& coefficient_friction_bodyB,
    bool is_block_with_4Spheres,  MultibodyPlant<double>* plant);

/// This method creates an inclined-plane A and uniform-density sphere (body B)
/// with user-defined mass, geometry, and surface friction properties and adds
/// them to an existing plant.  B's bottom surface contacts A's top surface.
/// Right-handed sets of orthonormal vectors Wx, Wy, Wz; Ax, Ay, Az; Bx, By, Bz;
/// are fixed in the world W, inclined-plane A, and sphere B, respectively.
/// The inclined-plane is discussed in AddInclinedPlaneAndGravityToPlant()
/// @param[in] gravity See AddInclinedPlaneAndGravityToPlant()
/// @param[in] inclined_plane_angle See AddInclinedPlaneAndGravityToPlant()
/// @param[in] is_inclined_plane_half_space See
/// AddInclinedPlaneAndGravityToPlant()
/// @param[in] inclined_plane_dimensions See
/// AddInclinedPlaneAndGravityToPlant()
/// @param[in] radiusB The radius of sphere B (meters), which must be positive.
/// @param[in] massB The mass of sphere B (kilograms), which must be positive.
/// @param[in] coefficient_friction_inclined_plane Coulomb's coefficient of
///   friction for inclined-plane A (sliding friction and static friction).
/// @param[in] coefficient_friction_bodyB Coulomb's coefficient of friction
///   data for body B's surfaces (sliding friction and static friction).
/// @param[in] is_inclined_plane_half_space This is `true` if inclined-plane A's
///   contact with block B is modeled using a half-space whereas `false` means
///   inclined-plane A is modeled using a box whose dimensions scale with B.
/// @param[out] plant Plant that will contain inclined-plane A and body B.
/// @note Decorative visual geometry is added to the sphere to facilitate
/// visualizing the sphere's rotation.
/// @throws std::exception if plant is nullptr or there is invalid data.
/// @pre plant must be registered with a scene graph.
void AddInclinedPlaneWithSphereToPlant(
    double gravity, double inclined_plane_angle,
    bool is_inclined_plane_half_space,
    const Vector3<double>& inclined_plane_dimensions,
    double radiusB, double massB,
    const CoulombFriction<double>& coefficient_friction_inclined_plane,
    const CoulombFriction<double>& coefficient_friction_bodyB,
    MultibodyPlant<double>* plant);

}  // namespace inclined_plane
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
