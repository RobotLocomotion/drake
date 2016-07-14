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

using Drake::BotVisualizer;
using Drake::SimulationOptions;
using Drake::cascade;

using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace cars {
namespace {

/** Driving Simulator
 * Usage:  car_sim_lcm_and_ros vehicle_model_file [world_model files ...]
 */
int do_main(int argc, const char* argv[]) {
  ::ros::init(argc, const_cast<char**>(argv), "car_sim_lcm_and_ros");

  // Initializes the communication layer.
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  // Instantiates a duration variable that will be set by the call to
  // CreateRigidBodySystem() below.
  double duration = std::numeric_limits<double>::infinity();

  // Initializes the rigid body system.
  auto rigid_body_sys = CreateRigidBodySystem(argc, argv, &duration);
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

  ::drake::ros::run_ros_vehicle_sim(sys, kStartTime, duration, x0, options);

  return 0;
}

}  // namespace
}  // namespace cars
}  // namespace examples
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::examples::cars::do_main(argc, argv);
}
