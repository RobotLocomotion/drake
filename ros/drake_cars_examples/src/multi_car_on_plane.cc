#include "ros/ros.h"

#include "drake/examples/Cars/car_simulation.h"
#include "drake/examples/Cars/gen/driving_command.h"
#include "drake/ros/parameter_server.h"
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

using drake::ros::systems::DrakeRosTfPublisher;
using drake::ros::systems::run_ros_vehicle_sim;

/**
 * This is the main method of the multi-car vehicle simulation. The vehicles
 * reside on a flat terrain.
 */
int DoMain(int argc, const char* argv[]) {
  ::ros::init(argc, const_cast<char**>(argv), "multi_car_on_plane");
  ::ros::NodeHandle node_handle;

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
  std::map<std::string, int> model_instances;

  // Obtains the number of vehicles to simulate.
  std::string car_count_parameter_name("car_count");
  int num_vehicles = GetROSParameter<int>(node_handle,
      car_count_parameter_name);

  const std::string model_name = "Prius";

  // Adds the vehicles to the rigid body system.
  for (int ii = 0; ii < num_vehicles; ++ii) {
    const std::string description_param_name = std::string("car_description_") +
        std::to_string(ii + 1);
    std::string description = GetROSParameter<std::string>(node_handle,
        description_param_name);
    RigidBodyTree::ModelToInstanceIDMap instance_ids;
    std::shared_ptr<RigidBodyFrame> weld_to_frame;
    rigid_body_sys->AddModelInstanceFromString(description,
        DrakeJoint::QUATERNION, weld_to_frame, &instance_ids);

    // The model file contains a single model. Get its model instance ID,
    // assign it a model instance name, and save both the instance ID and name
    // in sim_instance_ids.
    if (instance_ids.find(model_name) == instance_ids.end()) {
      throw std::runtime_error(
          "Failed to find a model instance whose model name is \"" +
          model_name + "\".");
    }
    model_instances[model_name + "_" + std::to_string(ii)] =
        instance_ids[model_name];
  }

  // std::cout << "==============================================" << std::endl;
  // for(auto it = sim_instance_ids.cbegin(); it != sim_instance_ids.cend();
  //     ++it) {
  //   std::cout << it->first << " " << it->second << "\n";
  // }
  // std::cout << std::endl;

  auto const& tree = rigid_body_sys->getRigidBodyTree();

  // Initializes and cascades all of the other systems.
  auto vehicle_sys = CreateVehicleSystem(rigid_body_sys);

  // auto visualizer =
  //     std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  // auto lidar_publisher = std::make_shared<
  //     ::drake::ros::SensorPublisherLidar<RigidBodySystem::StateVector>>(
  //     rigid_body_sys);

  // auto odometry_publisher = std::make_shared<
  //     ::drake::ros::SensorPublisherOdometry<RigidBodySystem::StateVector>>(
  //     rigid_body_sys);

  std::map<int, std::string> model_instance_names;
  // TODO(liang.fok): Fill in model_instance_names.
  // model_instance_names[model_instances["prius_1"]] = "prius_1";

  auto tf_publisher = std::make_shared<
      DrakeRosTfPublisher<RigidBodySystem::StateVector>>(tree,
          model_instance_names);

  // auto joint_state_publisher = std::make_shared<
  //     ::drake::ros::SensorPublisherJointState<RigidBodySystem::StateVector>>(
  //     rigid_body_sys);

  auto sys = cascade(vehicle_sys, tf_publisher);

  // auto sys =
  //     cascade(
  //         cascade(
  //             cascade(
  //                 cascade(
  //                     cascade(
  //                         vehicle_sys, visualizer),
  //                     lidar_publisher),
  //                 odometry_publisher),
  //             tf_publisher),
  //         joint_state_publisher);

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
