#include "ros/ros.h"

#include "drake/examples/Cars/car_simulation.h"
#include "drake/examples/Cars/gen/driving_command.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/pd_control_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/drakeAppUtil.h"
#include "drake_ros/ros_tf_publisher.h"
#include "drake_ros/ros_vehicle_system.h"
#include "drake_ros/ros_sensor_publisher_joint_state.h"
#include "drake_ros/ros_sensor_publisher_lidar.h"
#include "drake_ros/ros_sensor_publisher_odometry.h"

using drake::BotVisualizer;
using drake::SimulationOptions;
using drake::cascade;

using Eigen::VectorXd;

namespace drake {
namespace ros {
namespace cars {
namespace {

using drake::examples::cars::CreateRigidBodySystem;
using drake::examples::cars::CreateVehicleSystem;
using drake::examples::cars::GetCarSimulationDefaultOptions;

// Waits up to kMaxWaitTime for a particular parameter to exist on the ROS
// parameter server. Throws an std::runtime_error exception if the parameter
// fails to show up prior to this deadline.
void WaitForParameter(::ros::NodeHandle& ros_node_handle,
    const std::string& parameter_name) {
  // Waits for the parameter to exist on the ROS parameter server.
  const double kMaxWaitTime = 5.0;

  ::ros::Time begin_time = ::ros::Time::now();
  while (::ros::ok() && !ros_node_handle.hasParam(parameter_name)
      && (::ros::Time::now() - begin_time).toSec() < kMaxWaitTime) {
    ::ros::Duration(0.5).sleep(); // Sleeps for half a second.
  }

  if (!ros_node_handle.hasParam(parameter_name)) {
    throw std::runtime_error("ERROR: Failed to obtain parameter \"" +
        parameter_name + "\" from the ROS parameter server.");
  }
}

// Returns a string parameter from the ROS parameter server. Throws an
// `std::runtime_error` exception if it fails to obtain the parameter.
std::string GetStringParameter(::ros::NodeHandle &ros_node_handle,
    const std::string& parameter_name) {
  WaitForParameter(parameter_name, ros_node_handle)

  // Obtains the parameter from the ROS parameter server.
  std::string parameter;
  if (!ros_node_handle.getParam(parameter_name, parameter)) {
    throw std::runtime_error(
      "ERROR: Failed to obtain parameter \"" + parameter_name + "\" from "
      "the ROS parameter server. Please ensure such a parameter is loaded.");
  }

  return parameter;
}

// Returns an integer parameter from the ROS parameter server. Throws an
// `std::runtime_error` exception if it fails to obtain the parameter.
int GetIntParameter(::ros::NodeHandle &ros_node_handle,
    const std::string& parameter_name) {
  WaitForParameter(parameter_name, ros_node_handle)

  // Obtains the parameter from the ROS parameter server.
  int parameter;
  if (!ros_node_handle.getParam(parameter_name, parameter)) {
    throw std::runtime_error(
      "ERROR: Failed to obtain parameter \"" + parameter_name + "\" from "
      "the ROS parameter server. Please ensure such a parameter is loaded.");
  }

  return parameter;
}

/**
 * This is the main method of the multi-car vehicle simulation. The vehicles
 * reside on a flat terrain.
 */
int DoMain(int argc, const char* argv[]) {
  ::ros::init(argc, const_cast<char**>(argv), "multi_car_on_plane");

  // Initializes the communication layer.
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  // Instantiates a duration variable that will be set by the call to
  // CreateRigidBodySystem() below.
  double duration = std::numeric_limits<double>::infinity();

  // Initializes the rigid body system.
  auto rigid_body_sys = std::allocate_shared<RigidBodySystem>(
      Eigen::aligned_allocator<RigidBodySystem>());

  // Instantiates a data structure that maps model instance names to their model
  // instance IDs.
  std::unique_ptr<std::map<std::string, int>>
      model_instance_name_to_id_map(new std::map<std::string, int>());

  // Obtains the number of vehicles to simulate.
  int num_vehicles = GetIntParameter("car_count");

  // Adds the vehicles to the rigid body system.
  for (int ii = 0; ii < num_vehicles; ++ii) {
    const std::string description_param_name = car_description_ +
        std::to_string(ii+1);
    const std::string description = GetStringParameter(description_param_name);
    rigid_body_sys->AddModelInstanceFromString(description,
        DrakeJoint::QUATERNION);
  }

  auto const& tree = rigid_body_sys->getRigidBodyTree();

  // Initializes and cascades all of the other systems.
  auto vehicle_sys = CreateVehicleSystem(rigid_body_sys);

  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  auto lidar_publisher = std::make_shared<
      ::drake::ros::SensorPublisherLidar<RigidBodySystem::StateVector>>(
      rigid_body_sys);

  auto odometry_publisher = std::make_shared<
      ::drake::ros::SensorPublisherOdometry<RigidBodySystem::StateVector>>(
      rigid_body_sys);

  auto tf_publisher = std::make_shared<
      ::drake::ros::DrakeRosTfPublisher<RigidBodySystem::StateVector>>(tree);

  auto joint_state_publisher = std::make_shared<
      ::drake::ros::SensorPublisherJointState<RigidBodySystem::StateVector>>(
      rigid_body_sys);

  auto sys =
      cascade(
        cascade(
          cascade(
            cascade(
              cascade(
                vehicle_sys, visualizer),
              lidar_publisher),
            odometry_publisher),
          tf_publisher),
        joint_state_publisher);

  // Initializes the simulation options.
  SimulationOptions options = GetCarSimulationDefaultOptions();
  options.should_stop = [](double sim_time) {
    return !::ros::ok();
  };

  // Obtains a valid zero configuration for the vehicle.
  VectorXd x0 = VectorXd::Zero(rigid_body_sys->getNumStates());
  x0.head(tree->number_of_positions()) = tree->getZeroConfiguration();

  // Defines the start time of the simulation.
  const double kStartTime = 0;

  drake::ros::run_ros_vehicle_sim(sys, kStartTime, duration, x0, options);

  return 0;
}

}  // namespace
}  // namespace cars
}  // namespace ros
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::ros::cars::DoMain(argc, argv);
}
