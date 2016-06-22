#pragma once

#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "drake/drakeCars_export.h"
#include "drake/examples/Cars/gen/driving_command.h"
#include "drake/examples/Cars/gen/euler_floating_joint_state.h"
#include "drake/examples/Cars/gen/simple_car_state.h"
#include "drake/examples/Cars/trajectory_car.h"
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
namespace cars {

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
 * @param[out] duration The duration over which the simulation should run. The
 * simulation runs from time zero seconds to time \p duration seconds. If no
 * duration is specified in \p argv, this \p duration is set to be infinity.
 * A duration is specified in \p argv by the string "--duration" followed by a
 * floating point value.
 * @return A shared pointer to a rigid body system.
 */
DRAKECARS_EXPORT
std::shared_ptr<RigidBodySystem> CreateRigidBodySystem(int argc,
                                                       const char* argv[],
                                                       double* duration);

/**
 * Parses the simulation duration from the command line arguments. The duration
 * is specified by two sucessive tokens. The first token is "--duration" while
 * the second token is a string representation of a double value.
 *
 * @param[in] argc The number of command line arguments.
 * @param[in] argv An array of command line arguments.
 * @return the duration in seconds.
 */
DRAKECARS_EXPORT
double ParseDuration(int argc, const char* argv[]);

/**
 * Sets the penetration and friction parameters of the rigid body system.
 *
 * @param[in] rigid_body_sys The rigid body system to modify.
 */
DRAKECARS_EXPORT
void SetRigidBodySystemParameters(RigidBodySystem* rigid_body_sys);

/**
 * Adds a box-shaped terrain to the specified rigid body tree.
 *
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
DRAKECARS_EXPORT
void AddFlatTerrain(const std::shared_ptr<RigidBodyTree>& rigid_body_tree,
                    double box_size = 1000, double box_depth = 10);

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
 * Creates a TrajectoryCar system with a fixed trajectory.
 * The details of the trajectory are not documented / promised by this API.
 *
 * @param index Selects which pre-programmed trajectory to use.
 */
DRAKECARS_EXPORT
std::shared_ptr<TrajectoryCar> CreateTrajectoryCarSystem(int index);

/**
 * Creates a linear system to map NPC car state to the state vector of a
 * floating joint, allowing motion and steering in the x-y plane only.
 */
DRAKECARS_EXPORT
std::shared_ptr<Drake::AffineSystem<
  Drake::NullVector, SimpleCarState, EulerFloatingJointState>>
CreateSimpleCarVisualizationAdapter();

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

}  // namespace cars
}  // namespace examples
}  // namespace drake
