
#include <iostream>
#include <limits>

#include "drake/common/drake_path.h"
#include "drake/examples/Quadrotor/QuadrotorInput.h"
#include "drake/examples/Quadrotor/QuadrotorOutput.h"
#include "drake/system1/LCMSystem.h"
#include "drake/system1/LinearSystem.h"
#include "drake/system1/cascade_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/util/drakeAppUtil.h"

using namespace std;
using namespace drake;
using namespace Eigen;

using drake::systems::plants::joints::FloatingBaseType;
using drake::systems::plants::joints::kQuaternion;

int main(int argc, char* argv[]) {
  const size_t num_lidar_points = 100;

  SimulationOptions options;

  // Get the final time of the simulation.
  double final_time =
      argc >= 2 ? atof(argv[1]) : std::numeric_limits<double>::infinity();

  // Get the simulation's maximum acceptable delay in seconds.
  if (argc >= 3) {
    options.timeout_seconds = atof(argv[2]);
  }

  // Determine whether a warning should be printed or an exception should be
  // thrown if the simulation is delayed by more than the timeout threshold.
  if (commandLineOptionExists(argv, argv + argc, "--non-realtime")) {
    options.warn_real_time_violation = true;
  }

  cout << "Running simulation for " << final_time
       << " seconds with a maximum real-time delay of "
       << options.timeout_seconds << " seconds. Will "
       << (options.warn_real_time_violation ? "print warning"
                                            : "throw exception")
       << " if real-time max-delay is violated." << endl;

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  if (!lcm->good()) return 1;

  FloatingBaseType floating_base_type = kQuaternion;
  auto rigid_body_sys = make_shared<RigidBodySystem>();
  rigid_body_sys->AddModelInstanceFromFile(
      GetDrakePath() + "/examples/Quadrotor/warehouse.sdf", floating_base_type);

  rigid_body_sys->penetration_stiffness = 20.0;
  rigid_body_sys->penetration_damping = 2.0;

  auto const& tree = rigid_body_sys->getRigidBodyTree();
  rigid_body_sys->AddModelInstanceFromFile(
      GetDrakePath() + "/examples/Quadrotor/quadrotor_fla.urdf",
      floating_base_type);

  auto sensor_frame = tree->findFrame("body");

  auto accelerometer = make_shared<RigidBodyAccelerometer>(
      *rigid_body_sys, "accelerometer", sensor_frame);

  auto lidar2D = make_shared<RigidBodyDepthSensor>(
      *rigid_body_sys, "lidar2D", tree->findFrame("laser"), num_lidar_points,
      -3 * M_PI / 4, 3 * M_PI / 4, 30.0);
  auto gyroscope = make_shared<RigidBodyGyroscope>(*rigid_body_sys, "gyroscope",
                                                   sensor_frame);
  auto magnetometer = make_shared<RigidBodyMagnetometer>(
      *rigid_body_sys, "magnetometer", sensor_frame, 0.0);

  auto rangefinder = make_shared<RigidBodyDepthSensor>(
      *rigid_body_sys, "rangefinder", tree->findFrame("rangefinder"), 1, 0, 0,
      40.0);

  auto noise_model =
      make_shared<AdditiveGaussianNoiseModel<double, 3, Vector3d>>(0, 0.01);
  accelerometer->setNoiseModel(noise_model);
  accelerometer->setGravityCompensation(true);
  gyroscope->setNoiseModel(noise_model);
  magnetometer->setNoiseModel(noise_model);

  rigid_body_sys->addSensor(accelerometer);
  rigid_body_sys->addSensor(gyroscope);
  rigid_body_sys->addSensor(magnetometer);
  rigid_body_sys->addSensor(lidar2D);
  rigid_body_sys->addSensor(rangefinder);

  double box_width = 1000;
  double box_depth = 10;
  DrakeShapes::Box geom(Vector3d(box_width, box_width, box_depth));
  Isometry3d T_element_to_link = Isometry3d::Identity();
  T_element_to_link.translation() << 0, 0,
      -box_depth / 2;  // top of the box is at z=0
  RigidBody& world = tree->world();
  Vector4d color;
  color << 0.9297, 0.7930, 0.6758,
      1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
  world.AddVisualElement(
      DrakeShapes::VisualElement(geom, T_element_to_link, color));
  tree->addCollisionElement(
      DrakeCollision::Element(geom, T_element_to_link, &world), world,
      "terrain");
  tree->updateStaticCollisionElements();

  auto visualizer =
      make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  auto quad_control_to_rbsys_input =
      make_shared<Gain<QuadrotorInput, RigidBodySystem::InputVector>>(
          Eigen::Matrix4d::Identity());
  auto rbsys_output_to_quad_state =
      make_shared<Gain<RigidBodySystem::StateVector, QuadrotorOutput>>(
          Eigen::Matrix<double, 22 + num_lidar_points + 1,
                        22 + num_lidar_points + 1>::Identity());

  auto sys_with_lcm_input =
      cascade(quad_control_to_rbsys_input, rigid_body_sys);

  auto sys_with_vis = cascade(sys_with_lcm_input, visualizer);

  VectorXd x0 = VectorXd::Zero(rigid_body_sys->getNumStates());
  x0.head(tree->get_num_positions()) = tree->getZeroConfiguration();

  auto lcmio_with_vis = cascade(sys_with_vis, rbsys_output_to_quad_state);

  x0(0) = 0;
  x0(2) = 0.2;
  // x0(9) = 0.5; //yaw rate
  // x0(12) = 0.05; //Z dot

  runLCM(lcmio_with_vis, lcm, 0, final_time, x0, options);
}
