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

/** Driving Simulator
 * Usage:  car_sim_lcm_and_ros vehicle_model_file [world_model files ...]
 */
int do_main(int argc, const char* argv[]) {
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
  std::map<int, std::string> model_instance_names;
  // TODO(liang.fok): Until #3088 is resolved, the following two lines may be
  // overwritten since the first model added to the rigid body tree also takes
  // a model instance ID of zero. For this application, this is OK since the
  // consumers of model_instance_names does not need to know the model instance
  // name of the world.
  model_instance_names[RigidBodyTree::kWorldModelInstanceID] =
      RigidBodyTree::kWorldName;
  model_instance_names[model_instances["prius_1"]] = "prius";
  model_instance_names[model_instances["P1"]] = "stata_garage";

  // Initializes and cascades all of the other systems.
  auto vehicle_sys = CreateVehicleSystem(rigid_body_sys);

  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  auto lidar_publisher = std::make_shared<
      SensorPublisherLidar<RigidBodySystem::StateVector>>(
      rigid_body_sys, model_instance_names);

  auto odometry_publisher = std::make_shared<
      SensorPublisherOdometry<RigidBodySystem::StateVector>>(
      rigid_body_sys, model_instance_names);

  auto tf_publisher = std::make_shared<
      DrakeRosTfPublisher<RigidBodySystem::StateVector>>(tree,
          model_instance_names);

  auto joint_state_publisher = std::make_shared<
      SensorPublisherJointState<RigidBodySystem::StateVector>>(
      rigid_body_sys, model_instance_names);

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
  return drake::ros::cars::do_main(argc, argv);
}
