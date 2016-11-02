#include "ros/ros.h"

#include "drake/automotive/car_simulation.h"
#include "drake/automotive/gen/driving_command.h"
#include "drake/ros/systems/ros_tf_publisher.h"
#include "drake/ros/systems/ros_vehicle_system.h"
#include "drake/ros/systems/ros_sensor_publisher_joint_state.h"
#include "drake/ros/systems/ros_sensor_publisher_lidar.h"
#include "drake/ros/systems/ros_sensor_publisher_odometry.h"
#include "drake/system1/LCMSystem.h"
#include "drake/system1/LinearSystem.h"
#include "drake/system1/pd_control_system.h"
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

using drake::automotive::CreateRigidBodySystem;
using drake::automotive::CreateVehicleSystem;
using drake::automotive::GetCarSimulationDefaultOptions;
using drake::automotive::ParseDuration;

using drake::ros::systems::DrakeRosTfPublisher;
using drake::ros::systems::run_ros_vehicle_sim;
using drake::ros::systems::SensorPublisherJointState;
using drake::ros::systems::SensorPublisherLidar;
using drake::ros::systems::SensorPublisherOdometry;

/** Driving Simulator
 * Usage:  car_sim_lcm_and_ros vehicle_model_file [world_model files ...]
 */
int main(int argc, const char* argv[]) {
  ::ros::init(argc, const_cast<char**>(argv), "single_car_in_stata_garage");

  // Initializes the communication layer.
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  // Instantiates a duration variable that will be set by the call to
  // CreateRigidBodySystem() below.
  double duration = std::numeric_limits<double>::infinity();

  // Instantiates a data structure that maps model instance names to their model
  // instance IDs.
  drake::parsers::ModelInstanceIdTable model_instances;

  // Initializes the rigid body system.
  auto rigid_body_sys = CreateRigidBodySystem(argc, argv, &duration,
      &model_instances);

  auto const& tree = rigid_body_sys->getRigidBodyTree();

  // Initializes and cascades all of the other systems.
  auto vehicle_sys = CreateVehicleSystem(rigid_body_sys);

  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  auto lidar_publisher = std::make_shared<
      SensorPublisherLidar<RigidBodySystem::StateVector>>(
      rigid_body_sys);

  auto odometry_publisher = std::make_shared<
      SensorPublisherOdometry<RigidBodySystem::StateVector>>(
      rigid_body_sys);

  auto tf_publisher = std::make_shared<
      DrakeRosTfPublisher<RigidBodySystem::StateVector>>(tree);

  auto joint_state_publisher = std::make_shared<
      SensorPublisherJointState<RigidBodySystem::StateVector>>(
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
  x0.head(tree->get_num_positions()) = tree->getZeroConfiguration();

  // Defines the start time of the simulation.
  const double kStartTime = 0;

  // Starts the simulation.
  run_ros_vehicle_sim(sys, kStartTime, duration, x0, options);

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace ros
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::ros::automotive::main(argc, argv);
}
