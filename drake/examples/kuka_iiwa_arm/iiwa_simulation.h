#pragma once

#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <lcm/lcm-cpp.hpp>

#include "drake/drakeKukaIiwaArm_export.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/plants/BotVisualizer.h"
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
 * @param file_name A string with the filename (and path) of the URDF to be
 * loaded. Default argument is that of the IIWA Robot system with collision
 * flags.
 * @return A shared pointer to a rigid body system.
 */
DRAKEKUKAIIWAARM_EXPORT
std::shared_ptr<drake::RigidBodySystem> CreateKukaIiwaSystem(
    const std::string& file_name = std::string(
        "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf"));

/**
 * Creates a Bot Visualizer that can be cascaded with @p iiwa_system and
 * publishes LCM visualization messages using @p lcm.
 */
DRAKEKUKAIIWAARM_EXPORT
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
DRAKEKUKAIIWAARM_EXPORT
Eigen::VectorXd GenerateArbitraryIiwaInitialState();

/**
 * Returns the simulation options for use by the Kuka IIWA simulation.
 * @param initial_step_size Sets the initial step size for the simulate method.
 * Decrease if simulation is unstable.
 * @param real_time_factor Sets the real time factor for the simulation.
 * Increase if simstd::vector<double>ulation result renders faster than reality.
 */
DRAKEKUKAIIWAARM_EXPORT
drake::SimulationOptions SetupSimulation(double initial_step_size = 0.002,
                                         double real_time_factor = 0.0);

/**
 * Checks for joint position and velocity limit violations.
 * `std::runtime_error` is thrown if any limits are violated.
 */
DRAKEKUKAIIWAARM_EXPORT
void CheckLimitViolations(
    const std::shared_ptr<drake::RigidBodySystem> rigid_body_system,
    const Eigen::VectorXd& final_robot_state);

/**
 * Generates a demonstration joint trajectory by assigning constraints and
 * computing a corresponding inverse kinematic solution. The demonstration sets
 * up the arm to move from the (initial) straight up configuration to to reach
 * a position in the front of the robot and then repeat this motion twice.
 */
DRAKEKUKAIIWAARM_EXPORT
void GenerateIKDemoJointTrajectory(
    const std::shared_ptr<RigidBodyTree> iiwa_tree,
    Eigen::MatrixXd* joint_trajectories,
    std::vector<double>* time_stamps);

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
