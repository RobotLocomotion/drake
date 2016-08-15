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
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/RigidBodyTree.h"
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

using drake::parsers::ModelInstanceIdTable;

using drake::ros::systems::DrakeRosTfPublisher;
using drake::ros::systems::run_ros_vehicle_sim;
using drake::ros::systems::SensorPublisherJointState;

/**
 * Sits in a loop periodically publishing an identity transform for the
 * following frames to "world":
 *
 * 1. world_link
 * 2. Prius_1/world
 * 3. Prius_2/world
 * 4. Prius_3/world
 * 5. Prius_4/world
 * 6. Prius_5/world
 *
 * TODO(liang.fok): Remove these once #3081 is resolved. See:
 * https://github.com/RobotLocomotion/drake/issues/3081
 */
void WorldTfPublisher(int num_vehicles) {
  tf::TransformBroadcaster tf_broadcaster;

  std::vector<geometry_msgs::TransformStamped> transform_messages;

  geometry_msgs::TransformStamped world_link_message;
  world_link_message.header.frame_id = RigidBodyTree::kWorldName;
  world_link_message.child_frame_id = "ground_plane";
  world_link_message.transform.translation.x = 0;
  world_link_message.transform.translation.y = 0;
  world_link_message.transform.translation.z = 0;
  world_link_message.transform.rotation.w = 1;
  world_link_message.transform.rotation.x = 0;
  world_link_message.transform.rotation.y = 0;
  world_link_message.transform.rotation.z = 0;
  transform_messages.push_back(world_link_message);

  for (int ii = 0; ii < num_vehicles; ii++) {
    geometry_msgs::TransformStamped vehicle_message;
    vehicle_message.header.frame_id = RigidBodyTree::kWorldName;;
    vehicle_message.child_frame_id = std::string("Prius_") +
        std::to_string(ii + 1) + "/world";
    vehicle_message.transform.translation.x = 0;
    vehicle_message.transform.translation.y = 0;
    vehicle_message.transform.translation.z = 0;
    vehicle_message.transform.rotation.w = 1;
    vehicle_message.transform.rotation.x = 0;
    vehicle_message.transform.rotation.y = 0;
    vehicle_message.transform.rotation.z = 0;
    transform_messages.push_back(vehicle_message);
  }

  ::ros::Rate cycle_rate(10); // 10 Hz
  while (::ros::ok()) {
    tf_broadcaster.sendTransform(transform_messages);
    cycle_rate.sleep();
  }
}

/**
 * This is the main method of the multi-car vehicle simulation. The vehicles
 * reside on a flat terrain.
 */
int DoMain(int argc, const char* argv[]) {
  // Defines some constants.
  const std::string kModelName = "Prius";
  const std::string kWorldModelName = "planar_world";

  // Initializes ROS and a ROS node handle.
  ::ros::init(argc, const_cast<char**>(argv), "multi_cars_on_plane");
  ::ros::NodeHandle node_handle;

  // Initializes the LCM communication layer.
  // std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  // Initializes the rigid body system.
  auto rigid_body_sys = std::allocate_shared<RigidBodySystem>(
      Eigen::aligned_allocator<RigidBodySystem>());

  // Instantiates a data structure that maps model instance IDs to
  // application-defined model instance names.
  std::map<int, std::string> model_instance_name_table;

  // Obtains the number of vehicles to simulate.
  int num_vehicles = GetROSParameter<int>(node_handle, "car_count");

  // Adds the vehicles to the rigid body system.
  for (int ii = 0; ii < num_vehicles; ++ii) {
    const std::string description_parameter_name =
        std::string("car_description_") + std::to_string(ii + 1);
    std::string model_description = GetROSParameter<std::string>(node_handle,
        description_parameter_name);

    std::shared_ptr<RigidBodyFrame> weld_to_frame;
    ModelInstanceIdTable model_instance_id_table =
        rigid_body_sys->AddModelInstanceFromDescription(model_description,
            DrakeJoint::QUATERNION, weld_to_frame);

    // The model description contains a single model. Get its model instance ID,
    // assign it a model instance name, and save both in
    // model_instance_name_table.
    if (model_instance_id_table.find(kModelName) ==
        model_instance_id_table.end()) {
      throw std::runtime_error(
          "Failed to find a model instance whose model name is \"" +
          kModelName + "\".");
    }

    // Gets the model instance ID.
    int model_instance_id = model_instance_id_table[kModelName];

    // Derives a unique mdoel instance name.
    std::string model_instance_name = kModelName + "_" + std::to_string(ii+1);

    // Saves the model instance name in model_instance_name_table.
    model_instance_name_table[model_instance_id] = model_instance_name;
  }

  // Obtains and adds the world model to the RigidBodyTree. Then stores its
  // model instance ID and name in model_instance_name_table.
  std::string world_description = GetROSParameter<std::string>(node_handle,
      "world_description");
  ModelInstanceIdTable world_instance_id_table =
      rigid_body_sys->AddModelInstanceFromDescription(world_description,
          DrakeJoint::FIXED);
  int world_instance_id = world_instance_id_table[kWorldModelName];
  model_instance_name_table[world_instance_id] = kWorldModelName;

  // Spawns a thread publishing an identity transforms from the root of each
  // model to the world.
  //
  // TODO(liang.fok) Remove this once #3081 is resolved. See:
  // https://github.com/RobotLocomotion/drake/issues/3081
  std::thread worldTfPublisherThread(WorldTfPublisher, num_vehicles);

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

  auto tf_publisher = std::make_shared<
      DrakeRosTfPublisher<RigidBodySystem::StateVector>>(
          tree, model_instance_name_table);

  auto joint_state_publisher = std::make_shared<
      SensorPublisherJointState<RigidBodySystem::StateVector>>(
              rigid_body_sys, model_instance_name_table);

  auto sys =
      cascade(
        cascade(
            vehicle_sys, tf_publisher),
        joint_state_publisher);

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

  // Defines some simulation parameters.
  const double kStartTime = 0;
  double duration = std::numeric_limits<double>::infinity();

  drake::examples::cars::SetRigidBodySystemParameters(rigid_body_sys.get());

  // Starts the main simulation loop.
  run_ros_vehicle_sim(sys, kStartTime, duration, x0, options);

  worldTfPublisherThread.join();
  return 0;
}

}  // namespace
}  // namespace cars
}  // namespace ros
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::ros::cars::DoMain(argc, argv);
}
