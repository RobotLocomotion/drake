#pragma once

#include <memory>

#include <Eigen/Geometry>

#include "drake/drakeKukaIiwaArm_export.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/simulation_options.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/LinearSystem.h"

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
 * Generates a Bot visualizer and initialises the LCM to enable its usage.
 */
DRAKEKUKAIIWAARM_EXPORT
std::shared_ptr<BotVisualizer<RigidBodySystem::StateVector>>
    CreateKukaIiwaVisualizer(
    const std::shared_ptr<drake::RigidBodySystem> iiwa_system);

/**
 * Returns a vector corresponding to an arbitrarily fixed initial
 * state that can be used in demos and tests. This initial state
 * corresponds to an arbitrary initial joint configuration and with the
 * system at rest (0 velocities).
 */
DRAKEKUKAIIWAARM_EXPORT
Eigen::VectorXd ArbitraryIiwaInitialState();

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
