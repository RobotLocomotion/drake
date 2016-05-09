#pragma once

#include <string>
#include <memory>

#include <Eigen/Geometry>

#include "drake/drakeCar_export.h"
#include "drake/systems/pd_control_system.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/examples/Cars/gen/driving_command.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/Simulation.h"

using Drake::RigidBodySystem;
using Drake::PDControlSystem;
using Drake::CascadeSystem;
using Drake::Gain;
using Drake::DrivingCommand;
using Drake::SimulationOptions;

namespace drake {

/**
 * Parses the command line arguments and creates the rigid body system to be
 * simulated.
 *
 * @param[in] argc The number of command line arguments.
 * @param[in] argv an array of command line arguments.
 * @return A shared pointer to a rigid body system.
 */
std::shared_ptr<RigidBodySystem> CreateRigidBodySystem(int argc, char* argv[]);

/**
 * Creates a vehicle system by instantiating a PD controller and cascading it
 * with a rigid body system.
 *
 * @return The resulting vehicle system.
 */
std::shared_ptr<CascadeSystem<
    Gain<DrivingCommand, PDControlSystem<RigidBodySystem>::InputVector>,
    PDControlSystem<RigidBodySystem>>>
CreateVehicleSystem(std::shared_ptr<RigidBodySystem> rigid_body_sys);

/**
 * Sets the simulation options.
 *
 * @param[out] sim_options A pointer to where the simulation options should
 * be saved.
 */
void SetSimulationOptions(SimulationOptions* sim_options);

/**
 * Obtains a valid initial state of the system being simulated.
 *
 * @param[in] rigid_body_sys The rigid body system being simulated.
 * @return The initial state of the system.
 */
Eigen::VectorXd GetInitialState(
    std::shared_ptr<RigidBodySystem> rigid_body_sys);

}  // namespace drake
