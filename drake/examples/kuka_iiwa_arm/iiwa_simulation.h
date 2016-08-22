#pragma once

#include <memory>

#include <Eigen/Geometry>

#include "drake/drakeKukaIiwaArm_export.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/simulation_options.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
/**
 * Creates a rigid body system containing the IIWA arm, which can then be
 * simulated.
 *
 * It also sets the following simulation parameters:
 *
 * - penetration_stiffness = 3000.0
 * - penetration_damping = 0
 *
 * @return A shared pointer to a rigid body system.
 */
DRAKEKUKAIIWAARM_EXPORT
std::shared_ptr<drake::RigidBodySystem> CreateKukaIiwaSystem();

/**
 * Returns the simulation options for use by the Kuka IIWA simulation.
 */
DRAKEKUKAIIWAARM_EXPORT
drake::SimulationOptions SetupSimulation(double initial_step_size = 0.002);

/**
 * Checks for joint position and velocity limit violations.
 * `std::runtime_error` is thrown if any limits are violated.
 */
DRAKEKUKAIIWAARM_EXPORT
void CheckLimitViolations(
    const std::shared_ptr<drake::RigidBodySystem> rigid_body_system,
    const Eigen::VectorXd& final_robot_state);

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
