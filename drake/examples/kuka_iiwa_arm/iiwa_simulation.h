#pragma once

#include <memory>

#include <Eigen/Geometry>
#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_export.h"
#include "drake/system1/LinearSystem.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/system1/simulation_options.h"

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
DRAKE_EXPORT
std::shared_ptr<drake::RigidBodySystem> CreateKukaIiwaSystem();

/**
 * Creates a Bot Visualizer that can be cascaded with @p iiwa_system and
 * publishes LCM visualization messages using @p lcm.
 */
DRAKE_EXPORT
std::shared_ptr<BotVisualizer<RigidBodySystem::StateVector>>
    CreateKukaIiwaVisualizer(
    const std::shared_ptr<drake::RigidBodySystem> iiwa_system,
    const std::shared_ptr<lcm::LCM> lcm);

/**
 * Returns a vector corresponding to an arbitrarily fixed initial
 * state that can be used in demos and tests. This initial state
 * corresponds to an arbitrary initial joint configuration and with the
 * system at rest (0 velocities).
 */
DRAKE_EXPORT
Eigen::VectorXd GenerateArbitraryIiwaInitialState();

/**
 * Returns the simulation options for use by the Kuka IIWA simulation.
 */
DRAKE_EXPORT
drake::SimulationOptions SetupSimulation(double initial_step_size = 0.002);

/**
 * Checks for joint position and velocity limit violations.
 * `std::runtime_error` is thrown if any limits are violated.
 */
DRAKE_EXPORT
void CheckLimitViolations(
    const std::shared_ptr<drake::RigidBodySystem> rigid_body_system,
    const Eigen::VectorXd& final_robot_state);

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
