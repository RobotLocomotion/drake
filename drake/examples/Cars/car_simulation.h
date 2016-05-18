#pragma once

#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "drake/drakeCars_export.h"
#include "drake/examples/Cars/gen/driving_command.h"
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

/**
 * Parses the command line arguments and creates the rigid body system to be
 * simulated. The command line arguments consists of the vehicle's URDF or SDF
 * model file followed by an arbitrary number of model files representing things
 * the vehicle's environment.
 *
 * Note: This method will call `::exit(EXIT_FAILURE)` if it encounters a problem
 * parsing the input parameters.
 *
 * Note: If no flat terrain is specified by the input parameters, a flat
 * terrain is added automatically.
 *
 * @param[in] argc The number of command line arguments.
 * @param[in] argv An array of command line arguments.
 * @return A shared pointer to a rigid body system.
 */
DRAKECARS_EXPORT
std::shared_ptr<RigidBodySystem> CreateRigidBodySystem(int argc,
                                                       const char* argv[]);

/**
 * Creates a vehicle system by instantiating PD controllers for the actuators
 * in the model and cascading it with a rigid body system. The expected names
 * of the actuators are "steering", "right_wheel_joint", "left_wheel_joint".
 *
 * @param[in] rigid_body_sys The rigid body system.
 * @return The resulting vehicle system.
 */
DRAKECARS_EXPORT
std::shared_ptr<CascadeSystem<
    Gain<DrivingCommand, PDControlSystem<RigidBodySystem>::InputVector>,
    PDControlSystem<RigidBodySystem>>>
CreateVehicleSystem(std::shared_ptr<RigidBodySystem> rigid_body_sys);

/**
 * Returns the default simulation options for car simulations. The default
 * options include an initial step size of 5e-3 and a timeout of infinity.
 *
 * @return The default car simulation options.
 */
DRAKECARS_EXPORT
SimulationOptions GetCarSimulationDefaultOptions();

/**
 * Obtains a valid initial state of the system being simulated.
 *
 * @param[in] rigid_body_sys The rigid body system being simulated.
 * @return The initial state of the system.
 */
DRAKECARS_EXPORT
Eigen::VectorXd GetInitialState(const RigidBodySystem& rigid_body_sys);

}  // namespace drake
