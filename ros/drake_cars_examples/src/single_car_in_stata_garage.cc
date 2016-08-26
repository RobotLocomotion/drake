#include "ros/ros.h"

#include "drake/examples/Cars/car_simulation.h"
#include "drake/examples/Cars/gen/driving_command.h"
#include "drake/ros/systems/ros_tf_publisher.h"
#include "drake/ros/systems/ros_vehicle_system.h"
#include "drake/ros/systems/ros_sensor_publisher_joint_state.h"
#include "drake/ros/systems/ros_sensor_publisher_lidar.h"
#include "drake/ros/systems/ros_sensor_publisher_odometry.h"
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
namespace cars {
namespace {

using drake::examples::cars::CreateRigidBodySystem;
using drake::examples::cars::CreateVehicleSystem;
using drake::examples::cars::GetCarSimulationDefaultOptions;
using drake::examples::cars::ParseDuration;

using drake::ros::systems::DrakeRosTfPublisher;
using drake::ros::systems::run_ros_vehicle_sim;
using drake::ros::systems::SensorPublisherJointState;
using drake::ros::systems::SensorPublisherLidar;
using drake::ros::systems::SensorPublisherOdometry;

/**
 * This implements the main method of the single car simulation. The vehicle
 * resides within the Stata garage.
 */
int DoMain(int argc, const char* argv[]) {
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

  // Instantiates a map that converts model instance IDs to model instance
  // names.
  std::map<int, std::string> model_instance_name_table;
  // TODO(liang.fok): Once #3088 is resolved, include the model instance ID and
  // name of the world in model_instance_name_table.
  model_instance_name_table[model_instances["prius_1"]] = "prius";
  model_instance_name_table[model_instances["P1"]] = "stata_garage";

  std::map<int, std::string> model_instance_name_table_odometry;
  model_instance_name_table_odometry[model_instances["prius_1"]] = "prius";

  // Initializes and cascades all of the other systems.

  // The following method wraps the RigidBodySystem within a PD control system
  // block that adds PD controllers for each actuator within the
  // RigidBodySystem. It then cascades the PD control system block behind a
  // gain block and returns the resulting cascade.
  auto vehicle_sys = CreateVehicleSystem(rigid_body_sys);

  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  auto lidar_publisher = std::make_shared<
      SensorPublisherLidar<RigidBodySystem::StateVector>>(
      rigid_body_sys, model_instance_name_table);

  auto odometry_publisher = std::make_shared<
      SensorPublisherOdometry<RigidBodySystem::StateVector>>(
      rigid_body_sys, model_instance_name_table_odometry);

  auto tf_publisher = std::make_shared<
      DrakeRosTfPublisher<RigidBodySystem::StateVector>>(tree,
          model_instance_name_table);

  auto joint_state_publisher = std::make_shared<
      SensorPublisherJointState<RigidBodySystem::StateVector>>(
      rigid_body_sys, model_instance_name_table);

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

  // Starts the simulation.
  run_ros_vehicle_sim(sys, kStartTime, duration, x0, options);

  return 0;
}

}  // namespace
}  // namespace cars
}  // namespace ros
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::ros::cars::DoMain(argc, argv);
}
