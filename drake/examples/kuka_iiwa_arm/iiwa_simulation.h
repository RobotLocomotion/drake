#pragma once
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "drake/drakeKukaIIWAArm_export.h"
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
 * @param[out] duration The duration over which the simulation should run. The
 * simulation runs from time zero seconds to time \p duration seconds.
 */
DRAKEKUKAIIWAARM_EXPORT
Drake::SimulationOptions SetupSimulation(void);

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
