#include <cmath>
#include <cstdlib>
#include <memory>
#include <sstream>

#include "drake/Path.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/n_ary_state.h"
#include "drake/systems/n_ary_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodyTree.h"

#include "drake/examples/Cars/trivial_car.h"
#include "drake/examples/Cars/xyz_rpy.h"


int main(int argc, const char* argv[]) {

  int num_extra_cars = 100;
  if (argc == 2) {
    num_extra_cars = atoi(argv[1]);
    if (num_extra_cars < 0) {
      std::cerr << "The number of extra cars must be >=0.\n";
      std::exit(1);
    }
  }

  // Make a linear system to map NPC car state to the state vector of a
  // floating joint, allowing motion and steering in the x-y plane only.
  drake::XyzRpy<double> y0(0., 0., 0., 0., 0., M_PI / 2.);
  const int insize = drake::TrivialCarState<double>::RowsAtCompileTime;
  const int outsize = drake::XyzRpy<double>::RowsAtCompileTime;
  Eigen::Matrix<double, outsize, insize> D;
  D <<
      1, 0,  0,  // x
      0, 1,  0,  // y
      0, 0,  0,  // z
      0, 0,  0,  // roll
      0, 0,  0,  // pitch
      0, 0, -1;  // yaw
  auto car_vis_adapter = std::make_shared<
    Drake::AffineSystem<Drake::NullVector,
                        drake::TrivialCarState,
                        drake::XyzRpy> >(
      Eigen::MatrixXd::Zero(0, 0),
      Eigen::MatrixXd::Zero(0, insize),
      Eigen::VectorXd::Zero(0),
      Eigen::MatrixXd::Zero(outsize, 0), D,
      toEigen(y0));

  const std::string kSedanUrdf = Drake::getDrakePath() +
      "/examples/Cars/models/sedan.urdf";
  const std::string kBreadtruckUrdf = Drake::getDrakePath() +
      "/examples/Cars/models/breadtruck.urdf";

  // RigidBodyTree for visualization.
  auto world_tree = std::make_shared<RigidBodyTree>();

  // NarySystem for car 'physics'.
  //  U: ()
  //  X: ()
  //  Y: [(xy-position, heading), ...] per TrivialCarState
  auto cars_system = std::make_shared<drake::NArySystem<drake::TrivialCar> >();
  // NarySystem for car visualization.
  // BotVisualizer:
  //  U: [(xy-position, heading), ...] per TrivialCarState
  //  X: ()
  //  Y: [(x, y, z, roll, pitch, yaw), ...] per DrakeJoint::ROLLPITCHYAW per car
  auto cars_vis_adapter = std::make_shared<drake::NArySystem<decltype(car_vis_adapter)::element_type> >();
  // NB:  One could compose the other way as well (i.e., individually cascade
  // each TrivialCar with a car_vis_adapter, and then stack each of those pairs
  // into a single NArySystem).

  // Create one Sedan.
  world_tree->addRobotFromURDF(kSedanUrdf,
                               DrakeJoint::ROLLPITCHYAW,
                               nullptr /*weld_to_frame*/);
  // TODO maddog  Hmm... it appears that drake_visualizer wants unique names,
  //              on *links*, otherwise only one of the same-named links will
  //              get updated joint parameters.
  world_tree->bodies.back()->linkname = "sedan1";
  auto sedan_system = std::make_shared<drake::TrivialCar>(0., 5., 0.3, 2.0);
  cars_system->addSystem(sedan_system);
  cars_vis_adapter->addSystem(car_vis_adapter);

  // Create one Breadtruck.
  world_tree->addRobotFromURDF(kBreadtruckUrdf,
                               DrakeJoint::ROLLPITCHYAW,
                               nullptr /*weld_to_frame*/);
  world_tree->bodies.back()->linkname = "breadtruck1";
  auto breadtruck_system = std::make_shared<drake::TrivialCar>(15., 0., 0.7, 10.0);
  cars_system->addSystem(breadtruck_system);
  cars_vis_adapter->addSystem(car_vis_adapter);

  // Create another Sedan.
  world_tree->addRobotFromURDF(kSedanUrdf,
                               DrakeJoint::ROLLPITCHYAW,
                               nullptr /*weld_to_frame*/);
  world_tree->bodies.back()->linkname = "sedan2";
  auto sedan2_system = std::make_shared<drake::TrivialCar>(-5., 0., -0.3, -2.0);
  cars_system->addSystem(sedan2_system);
  cars_vis_adapter->addSystem(car_vis_adapter);

  // Meh, need a lot more cars!
  for (int i = 0; i < num_extra_cars; ++i) {
    world_tree->addRobotFromURDF(
        (i % 2) ? kSedanUrdf : kBreadtruckUrdf,
        DrakeJoint::ROLLPITCHYAW,
        nullptr /*weld_to_frame*/);
    std::ostringstream name;
    name << "car-" << i;
    world_tree->bodies.back()->linkname = name.str();
    auto system = std::make_shared<drake::TrivialCar>(
        // Hackery to spread them around a little on the plane.
        (i % 19 - 10) * 10.,
        ((i + 11) % 23 - 11) * 10.,
        0.0,
        ((i % 10) - 5) * 2.0);
    cars_system->addSystem(system);
    cars_vis_adapter->addSystem(car_vis_adapter);
  }

  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
  auto visualizer =
      std::make_shared<Drake::BotVisualizer<
        decltype(cars_vis_adapter)::element_type::OutputVector> >(
            lcm, world_tree);

  // Compose the system together from the parts.
  auto the_system = Drake::cascade(Drake::cascade(
      cars_system, cars_vis_adapter), visualizer);

  decltype(the_system)::element_type::StateVector<double> initial_state;
  Drake::SimulationOptions options;
  options.realtime_factor = 0.;
  runLCM(the_system, lcm,
      0, std::numeric_limits<double>::infinity(), // timespan
      initial_state,
      options);

  return 0;
}
