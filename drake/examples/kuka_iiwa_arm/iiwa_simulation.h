#pragma once
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "drake/systems/LinearSystem.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/pd_control_system.h"
#include "drake/systems/plants/RigidBodySystem.h"

using Drake::RigidBodySystem;
using Drake::PDControlSystem;
using Drake::CascadeSystem;
using Drake::Gain;
using Drake::SimulationOptions;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
/**
 * Ceates the rigid body system of the IIWA arm to be
 * simulated. The command line arguments consists of the vehicle's URDF or SDF
 * model file followed by an arbitrary number of model files representing things
 * the vehicle's environment.
 *
 * Current version will not use arguments.
 *
 * @param[in] argc The number of command line arguments.
 * @param[in] argv An array of command line arguments.
 * @return A shared pointer to a rigid body system.
 *
 * Also sets penetration_stiffness = 3000.0
 * and, penetration_damping = 0;
 */
std::shared_ptr<RigidBodySystem> CreateIIWAArmSystem(void);

/** Adds a box-shaped terrain to the specified rigid body tree.
 * The X, Y, and Z axes of the box matches the X, Y, and Z-axis of the world.
 * The length and width of the box is aligned with X and Y and are \p box_size
 * long. The depth of the box is aligned with Z and is \p box_depth long. The
 * top surface of the box is at Z = 0.
 *
 * @param[in] rigid_body_tree The rigid body tree to which to add the terrain.
 * @param[in] box_size The length and width of the terrain aligned with the
 * world's X and Y axes.
 * @param[in] box_depth The depth of the terrain aligned with the world's Z
 * axis. Note that regardless of how deep the terrain is, the top surface of the
 * terrain will be at Z = 0.
 */
void SetupWorld(const std::shared_ptr<RigidBodyTree>& rigid_body_tree,
                double box_size = 3.0, double box_depth = 0.2);

/**
 * @param[out] duration The duration over which the simulation should run. The
 * simulation runs from time zero seconds to time \p duration seconds.
 */
Drake::SimulationOptions SetupSimulation(void);

/**
 * This method simply throws an error if limits are generated.
 */
// void ValidateSimulation(const
// std::shared_ptr<RobotStateTap<RigidBodySystem::StateVector>>&
// robot_state_tap,
//                        const std::shared_ptr<RigidBodySystem>& iiwa_system);
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
