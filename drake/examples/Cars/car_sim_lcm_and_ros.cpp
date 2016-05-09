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

using namespace std;
using namespace Eigen;
using namespace Drake;



/** Driving Simulator
 * Usage:  simulateLCM vehicle_model_file [world_model files ...]
 */

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "drake_car_sim_lcm_and_ros");

  auto rigid_body_sys = drake::CreateRigidBodySystem(argc, argv);
  auto const& tree = rigid_body_sys->getRigidBodyTree();

  MatrixXd Kp(getNumInputs(*rigid_body_sys), tree->num_positions),
      Kd(getNumInputs(*rigid_body_sys), tree->num_velocities);
  Matrix<double, Eigen::Dynamic, 3> map_driving_cmd_to_x_d(
      tree->num_positions + tree->num_velocities, 3);
  {  // setup PD controller for throttle and steering
    double kpSteering = 400, kdSteering = 80, kThrottle = 100;
    Kp.setZero();
    Kd.setZero();
    map_driving_cmd_to_x_d.setZero();

    for (int actuator_idx = 0; actuator_idx < tree->actuators.size();
         actuator_idx++) {
      const std::string& actuator_name = tree->actuators[actuator_idx].name;

      if (actuator_name == "steering") {
        auto const& b = tree->actuators[actuator_idx].body;
        Kp(actuator_idx, b->position_num_start) = kpSteering;  // steering
        Kd(actuator_idx, b->velocity_num_start) = kdSteering;  // steeringdot
        map_driving_cmd_to_x_d(b->position_num_start, 0) =
            1;  // steering command

      } else if (actuator_name == "right_wheel_joint" ||
                 actuator_name == "left_wheel_joint") {
        auto const& b = tree->actuators[actuator_idx].body;
        Kd(actuator_idx, b->velocity_num_start) = kThrottle;  // throttle
        map_driving_cmd_to_x_d(tree->num_positions + b->velocity_num_start, 1) =
            20;  // throttle (velocity) command
        map_driving_cmd_to_x_d(tree->num_positions + b->velocity_num_start, 2) =
            -20;  // braking (velocity) command
      }
    }
  }

  auto vehicle_with_pd =
      make_shared<PDControlSystem<RigidBodySystem>>(rigid_body_sys, Kp, Kd);

  auto vehicle_sys = cascade(
      make_shared<
          Gain<DrivingCommand, PDControlSystem<RigidBodySystem>::InputVector>>(
          map_driving_cmd_to_x_d),
      vehicle_with_pd);

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();

  auto bot_visualizer =
      make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  auto lidar_visualizer =
      make_shared<drake::systems::plants::SensorVisualizerLidar<
          RigidBodySystem::StateVector>>(rigid_body_sys);

  auto tf_publisher = make_shared<drake::systems::plants::DrakeRosTfPublisher<
      RigidBodySystem::StateVector>>(tree);

  auto sys =
      cascade(cascade(cascade(vehicle_sys, bot_visualizer), lidar_visualizer),
              tf_publisher);

  SimulationOptions options = default_simulation_options;
  rigid_body_sys->penetration_stiffness = 5000.0;
  rigid_body_sys->penetration_damping =
      rigid_body_sys->penetration_stiffness / 10.0;
  rigid_body_sys->friction_coefficient = 10.0;  // essentially infinite friction
  options.initial_step_size = 5e-3;
  options.timeout_seconds = numeric_limits<double>::infinity();

  VectorXd x0 = VectorXd::Zero(rigid_body_sys->getNumStates());
  x0.head(tree->num_positions) = tree->getZeroConfiguration();
  // todo:  call getInitialState instead?  (but currently, that would require
  // snopt).  needs #1627
  // I'm getting away without it, but might be generating large internal forces
  // initially as the ackerman constraint (hopefully) gets enforced by the
  // stabilization terms.

  runLCM(sys, lcm, 0, std::numeric_limits<double>::infinity(), x0, options);
  //  simulate(*sys, 0, std::numeric_limits<double>::infinity(), x0, options);

  return 0;
}
