#include "drake/examples/Cars/car.h"
#include "drake/examples/Cars/gen/driving_command.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/pd_control_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/drakeAppUtil.h"
#include "drake/systems/plants/sensor_visualizer_lidar.h"
#include "drake/systems/plants/drake_ros_tf_publisher.h"

using Drake::BotVisualizer;
using Drake::DrivingCommand;
using Drake::Gain;
using Drake::SimulationOptions;

using Eigen::VectorXd;

/** Driving Simulator
 * Usage:  simulateLCM vehicle_model_file [world_model files ...]
 */

int main(int argc, char* argv[]) {
  // Initializes communication layer(s).
  ros::init(argc, argv, "drake_car_sim_lcm_and_ros");
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  // Initializes the rigid body system.
  auto rigid_body_sys = drake::CreateRigidBodySystem(argc, argv);
  auto const& tree = rigid_body_sys->getRigidBodyTree();

  // Initializes and cascades all of the other systems.
  auto vehicle_sys = drake::CreateVehicleSystem(rigid_body_sys);

  auto bot_visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  auto lidar_visualizer =
      std::make_shared<drake::systems::plants::SensorVisualizerLidar<
          RigidBodySystem::StateVector>>(rigid_body_sys);

  auto tf_publisher =
      std::make_shared<drake::systems::plants::DrakeRosTfPublisher<
          RigidBodySystem::StateVector>>(tree);

  auto sys =
      cascade(cascade(cascade(vehicle_sys, bot_visualizer), lidar_visualizer),
              tf_publisher);

  // Initializes the simulation options.
  SimulationOptions options;
  drake::SetSimulationOptions(&options);

  // Starts the simulation.
  Drake::runLCM(sys, lcm, 0, std::numeric_limits<double>::infinity(),
                drake::GetInitialState(rigid_body_sys), options);
  return 0;
}
