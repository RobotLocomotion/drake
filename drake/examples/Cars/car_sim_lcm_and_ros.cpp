#include "drake/examples/Cars/car.h"
#include "drake/examples/Cars/gen/driving_command.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/pd_control_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/drakeAppUtil.h"
#include "drake/ros/sensor_visualizer_lidar.h"
#include "drake/ros/drake_ros_tf_publisher.h"

using Drake::BotVisualizer;
using Drake::Gain;
using Drake::SimulationOptions;

using Eigen::VectorXd;

namespace drake {
namespace {

int do_main(int argc, const char* argv[]) {
  // Initializes communication layer(s).
  ros::init(argc, const_cast<char**>(argv), "drake_car_sim_lcm_and_ros");
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
  SetSimulationOptions(&options);

  // Obtains the duration of the simulation
  double duration = std::numeric_limits<double>::infinity();
  char* duration_string = getCommandLineOption(
      const_cast<char**>(argv), const_cast<char**>(argv) + argc, "--duration");
  if (duration_string) duration = atof(duration_string);

  std::cout << "Running simulation for duration " << duration << std::endl;

  // Starts the simulation.
  Drake::runLCM(sys, lcm, 0, duration, drake::GetInitialState(rigid_body_sys),
                options);
  return 0;
}

}  // namespace
}  // namespace drake

int main(int argc, const char* argv[]) { return drake::do_main(argc, argv); }
