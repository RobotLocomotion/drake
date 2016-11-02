#pragma once

#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/system1_vector.h"
#include "drake/common/drake_export.h"
#include "drake/system1/LinearSystem.h"
#include "drake/system1/Simulation.h"
#include "drake/system1/cascade_system.h"
#include "drake/system1/pd_control_system.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/lcmt_driving_command_t.hpp"

namespace drake {
namespace automotive {

/// Compatibility class for System 1 code.
// TODO(jwnimmer-tri) Remove me.
template <typename T>
class DRAKE_EXPORT DrivingCommand1 :
    public System1Vector<DrivingCommand<T>, T> {
 public:
  DrivingCommand1() {}

  explicit DrivingCommand1(const System1Vector<DrivingCommand<T>, T>& other) :
      System1Vector<DrivingCommand<T>, T>(other) {
  }

  /// @name Implements the LCMVector concept.
  //@{
  typedef drake::lcmt_driving_command_t LCMMessageType;
  static std::string channel() { return "DRIVING_COMMAND"; }
  //@}
};

/**
 * Prints the usage instructions to std::cout.
 */
DRAKE_EXPORT
void PrintUsageInstructions(const std::string& executable_name);

/**
 * Parses the command line arguments and creates the rigid body system to be
 * simulated. The command line arguments consists of the vehicle's URDF or SDF
 * model file followed by an arbitrary number of model files representing things
 * the vehicle's environment.
 *
 * <B>Note 1:</B> This method will call `::exit(EXIT_FAILURE)` if it encounters
 * a problem parsing the input parameters.
 *
 * <B>Note 2:</B> If no flat terrain is specified by the input parameters, a
 * flat terrain is added automatically.
 *
 * <B>Note 3:</B> The name of each model in the set of URDF and SDF files listed
 * in @p argv must be unique within this set. In other words, a particular model
 * name can appear at most once among all of the models defined in the URDF and
 * SDF files listed in @p argv.
 *
 * @param[in] argc The number of command line arguments.
 *
 * @param[in] argv An array of command line arguments.
 *
 * @param[out] duration The duration over which the simulation should run. The
 * simulation runs from time zero seconds to time @p duration seconds. If no
 * duration is specified in @p argv, this @p duration is set to be infinity.
 * A duration is specified in @p argv by the string "--duration" followed by a
 * floating point value. This parameter is optional. If it is nullptr, the
 * duration is not saved.
 *
 * @param[out] model_instance_id_table A pointer to where the model instance
 * IDs are to be stored.
 *
 * @return A shared pointer to a rigid body system.
 */
DRAKE_EXPORT
std::shared_ptr<RigidBodySystem> CreateRigidBodySystem(
    int argc, const char* argv[], double* duration,
    drake::parsers::ModelInstanceIdTable* model_instance_id_table);

/**
 * Checks the command line arguments looking for a "--duration" flag followed
 * by a double value, which indicates the length of time in seconds the
 * simulation should run. If no such flag is found, return
 * `std::numeric_limits<double>::infinity()`.
 *
 * @param[in] argc The number of command line arguments.
 * @param[in] argv An array of command line arguments.
 * @return The duration in seconds.
 * @throws std::runtime_error if "--duration" exists in @p argv but is not
 * followed by a double value.
 */
DRAKE_EXPORT
double ParseDuration(int argc, const char* argv[]);

/**
 * Sets the penetration and friction parameters of the rigid body system.
 *
 * @param[in] rigid_body_sys The rigid body system to modify.
 */
DRAKE_EXPORT
void SetRigidBodySystemParameters(RigidBodySystem* rigid_body_sys);

/**
 * Adds a box-shaped terrain to the specified rigid body tree. This directly
 * modifies the existing world rigid body within @p rigid_body_tree and thus
 * does not need to return a model name or model_instance_id value.
 *
 * The X, Y, and Z axes of the box matches the X, Y, and Z-axis of the world.
 * The length and width of the box is aligned with X and Y and are @p box_size
 * long. The depth of the box is aligned with Z and is @p box_depth long. The
 * top surface of the box is at Z = 0.
 *
 * @param[in] rigid_body_tree The rigid body tree to which to add the terrain.
 *
 * @param[in] box_size The length and width of the terrain aligned with the
 * world's X and Y axes.
 *
 * @param[in] box_depth The depth of the terrain aligned with the world's Z
 * axis. Note that regardless of how deep the terrain is, the top surface of the
 * terrain will be at Z = 0.
 */
DRAKE_EXPORT
void AddFlatTerrainToWorld(
    const std::shared_ptr<RigidBodyTree>& rigid_body_tree,
    double box_size = 1000, double box_depth = 10);

/**
 * Creates a vehicle system by instantiating PD controllers for the actuators
 * in the model and cascading it with a rigid body system. The expected names
 * of the actuators are "steering", "right_wheel_joint", "left_wheel_joint".
 *
 * @param[in] rigid_body_sys The rigid body system.
 * @return The resulting vehicle system.
 */
DRAKE_EXPORT
std::shared_ptr<CascadeSystem<
    Gain<DrivingCommand1, PDControlSystem<RigidBodySystem>::InputVector>,
    PDControlSystem<RigidBodySystem>>>
CreateVehicleSystem(std::shared_ptr<RigidBodySystem> rigid_body_sys);

/**
 * Returns the default simulation options for car simulations. The default
 * options include an initial step size of 5e-3 and a timeout of infinity.
 *
 * @return The default car simulation options.
 */
DRAKE_EXPORT
SimulationOptions GetCarSimulationDefaultOptions();

/**
 * Obtains a valid initial state of the system being simulated.
 *
 * @param[in] rigid_body_sys The rigid body system being simulated.
 * @return The initial state of the system.
 */
DRAKE_EXPORT
Eigen::VectorXd GetInitialState(const RigidBodySystem& rigid_body_sys);

/// Compatibility function for System 1 code.
// TODO(jwnimmer-tri) Remove me.
template <typename ScalarType>
bool encode(const double& t, const DrivingCommand<ScalarType>& wrap,
            // NOLINTNEXTLINE(runtime/references)
            drake::lcmt_driving_command_t& msg) {
  msg.timestamp = static_cast<int64_t>(t * 1000);
  msg.steering_angle = wrap.steering_angle();
  msg.throttle = wrap.throttle();
  msg.brake = wrap.brake();
  return true;
}

/// Compatibility function for System 1 code.
// TODO(jwnimmer-tri) Remove me.
template <typename ScalarType>
bool decode(const drake::lcmt_driving_command_t& msg,
            // NOLINTNEXTLINE(runtime/references)
            double& t,
            // NOLINTNEXTLINE(runtime/references)
            DrivingCommand<ScalarType>& wrap) {
  t = static_cast<double>(msg.timestamp) / 1000.0;
  wrap.set_steering_angle(msg.steering_angle);
  wrap.set_throttle(msg.throttle);
  wrap.set_brake(msg.brake);
  return true;
}

}  // namespace automotive
}  // namespace drake
