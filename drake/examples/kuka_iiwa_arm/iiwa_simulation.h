#pragma once
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "drake/drakeKukaIiwaArm_export.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/plants/RigidBodySystem.h"

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
std::shared_ptr<Drake::RigidBodySystem> CreateKukaIiwaSystem();

/**
 * Returns the simulation options for use by the Kuka IIWA simulation.
 */
DRAKEKUKAIIWAARM_EXPORT
Drake::SimulationOptions SetupSimulation();

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
