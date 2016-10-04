#include <cmath>

#include "ros/ros.h"
#include "ros/console.h"

#include "rosgraph_msgs/Clock.h"

#include "drake/automotive/car_simulation.h"
#include "drake/automotive/gen/driving_command.h"
#include "drake/ros/simulation_abort_function.h"
#include "drake/ros/systems/ros_clock_publisher.h"
#include "drake/ros/systems/ros_sensor_publisher_joint_state.h"
#include "drake/ros/systems/ros_sensor_publisher_lidar.h"
#include "drake/ros/systems/ros_sensor_publisher_odometry.h"
#include "drake/ros/systems/ros_tf_publisher.h"
#include "drake/ros/systems/ros_vehicle_system.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/pd_control_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/drakeAppUtil.h"

using drake::BotVisualizer;
using drake::SimulationOptions;
using drake::cascade;

using Eigen::VectorXd;

namespace drake {
namespace ros {
namespace automotive {
namespace {

using drake::automotive::CreateVehicleSystem;
using drake::automotive::GetCarSimulationDefaultOptions;
using drake::automotive::ParseDuration;
using drake::automotive::SetRigidBodySystemParameters;

using drake::parsers::ModelInstanceIdTable;

using drake::ros::systems::RosTfPublisher;
using drake::ros::systems::RosClockPublisher;
using drake::ros::systems::RosSensorPublisherJointState;
using drake::ros::systems::RosSensorPublisherLidar;
using drake::ros::systems::RosSensorPublisherOdometry;

using drake::ros::systems::run_ros_vehicle_sim;

using drake::systems::plants::joints::kFixed;
using drake::systems::plants::joints::kQuaternion;


// Initializes the RigidBodySystem by setting the collision gains, adding the
// vehicle model, and adding the world model to @p rigid_body_system.
ModelInstanceIdTable InitRigidBodySystem(RigidBodySystem* rigid_body_system) {
  // Sets the desired contact penetration stiffness and damping gains in the
  // RigidBodySystem. This is done by first setting them to be the default
  // values and then reading the values off the ROS parameter server.
  SetRigidBodySystemParameters(rigid_body_system);
  rigid_body_system->penetration_stiffness =
      GetRosParameterOrThrow<double>("penetration_stiffness");
  rigid_body_system->penetration_damping =
      GetRosParameterOrThrow<double>("penetration_damping");

  // Adds the vehicle model instance to the RigidBodySystem.
  std::string vehicle_filename =
      GetRosParameterOrThrow<std::string>("car_filename");

  ModelInstanceIdTable vehicle_instance_id_table =
      rigid_body_system->AddModelInstanceFromFile(vehicle_filename,
          kQuaternion);

  // Verifies that only one vehicle was added to the world.
  if (vehicle_instance_id_table.size() != 1) {
    throw std::runtime_error(
        "More than one vehicle model was added to the world.");
  }

  // Instantiates a model_instance_id_table and saves the vehicle's model
  // instance ID in the table.
  ModelInstanceIdTable model_instance_id_table = vehicle_instance_id_table;

  // Adds the world to the RigidBodySystem. Updates model_instance_id_table
  // with the ID of the world model.
  std::string world_filename =
      GetRosParameterOrThrow<std::string>("world_filename");
  ModelInstanceIdTable world_instance_id_table =
      rigid_body_system->AddModelInstanceFromFile(world_filename,
          kFixed);
  drake::parsers::AddModelInstancesToTable(world_instance_id_table,
      &model_instance_id_table);

  return model_instance_id_table;
}

// This implements the main method of the single car simulation. The vehicle
// resides within the Stata garage.
int do_main(int argc, const char* argv[]) {
  ::ros::init(argc, const_cast<char**>(argv), "single_car_in_stata_garage");
  ::ros::start();

  // Sets the log level to be INFO.
  if (::ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ::ros::console::levels::Info)) {
    ::ros::console::notifyLoggerLevelsChanged();
  }

  // Initializes the rigid body system.
  auto rigid_body_sys = std::allocate_shared<RigidBodySystem>(
      Eigen::aligned_allocator<RigidBodySystem>());

  ModelInstanceIdTable model_instance_id_table =
      InitRigidBodySystem(rigid_body_sys.get());

  auto const& tree = rigid_body_sys->getRigidBodyTree();

  // Instantiates a map that converts model instance IDs to model instance
  // names.
  std::map<int, std::string> model_instance_name_table;
  // TODO(liang.fok): Once #3088 is resolved, include the model instance ID and
  // name of the world in model_instance_name_table.
  model_instance_name_table[model_instance_id_table["prius_1"]] = "prius";
  model_instance_name_table[model_instance_id_table["P1"]] = "stata_garage";

  std::map<int, std::string> model_instance_name_table_odometry;
  model_instance_name_table_odometry[model_instance_id_table["prius_1"]]
      = "prius";

  // Obtains the gains to be used by the steering and throttle controllers.
  double steering_kp = GetRosParameterOrThrow<double>("steering_kp");
  double steering_kd = GetRosParameterOrThrow<double>("steering_kd");
  double throttle_k = GetRosParameterOrThrow<double>("throttle_k");

  // Initializes and cascades all of the other systems.

  // Wraps the RigidBodySystem within a PD control system that adds PD
  // controllers for each actuator within the RigidBodySystem. It then cascades
  // the PD control system block behind a gain block and returns the resulting
  // cascade.
  auto vehicle_sys = CreateVehicleSystem(rigid_body_sys, steering_kp,
      steering_kd, throttle_k);

  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  auto lidar_publisher = std::make_shared<
      RosSensorPublisherLidar<RigidBodySystem::StateVector>>(
      rigid_body_sys, model_instance_name_table);

  auto odometry_publisher = std::make_shared<
      RosSensorPublisherOdometry<RigidBodySystem::StateVector>>(
      rigid_body_sys, model_instance_name_table_odometry);

  auto tf_publisher = std::make_shared<
      RosTfPublisher<RigidBodySystem::StateVector>>(tree,
          model_instance_name_table);

  auto joint_state_publisher = std::make_shared<
      RosSensorPublisherJointState<RigidBodySystem::StateVector>>(
      rigid_body_sys, model_instance_name_table);

  auto clock_publisher =
      std::make_shared<RosClockPublisher<RigidBodySystem::StateVector>>();

  auto sys =
      cascade(
        cascade(
          cascade(
            cascade(
              cascade(
                cascade(
                  vehicle_sys, visualizer),
                lidar_publisher),
              odometry_publisher),
            tf_publisher),
          joint_state_publisher),
        clock_publisher);

  // Initializes the simulation options.
  SimulationOptions options = GetCarSimulationDefaultOptions();
  AddAbortFunction(&options);

  options.initial_step_size =
      GetRosParameterOrThrow<double>("initial_step_size");

  // Obtains a valid zero configuration for the vehicle.
  VectorXd x0 = VectorXd::Zero(rigid_body_sys->getNumStates());
  x0.head(tree->get_num_positions()) = tree->getZeroConfiguration();

  // Defines the start time of the simulation.
  const double kStartTime = 0;

  ROS_INFO("Obtaining duration...");

  // Instantiates a duration variable. This specifies how long the simulation
  // will run.
  double duration = GetRosParameterOrDefault<double>(
      "simulation_duration", std::numeric_limits<double>::infinity(), 1);

  ROS_INFO_STREAM("Using:" << std::endl
      << " - duration = " << duration
      << std::endl
      << " - penetration_stiffness = " << rigid_body_sys->penetration_stiffness
      << std::endl
      << " - penetration_damping = " << rigid_body_sys->penetration_damping
      << std::endl
      << "  - steering_kp = " << steering_kp
      << std::endl
      << "  - steering_kd = " << steering_kd
      << std::endl
      << "  - throttle_k = " << throttle_k
      << std::endl
      << "  - initial_step_size = " << options.initial_step_size);

  // Starts the simulation.
  run_ros_vehicle_sim(sys, kStartTime, duration, x0, options);

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace ros
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::ros::automotive::do_main(argc, argv);
}
