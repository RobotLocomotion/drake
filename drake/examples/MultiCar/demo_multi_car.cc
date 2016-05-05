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
#include "drake/systems/Simulation.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodyTree.h"

#include "drake/examples/MultiCar/trivial_car.h"
#include "drake/examples/MultiCar/xyz_rpy.h"


int main(int argc, const char* argv[]) {

  int num_extra_cars = 100;
  if (argc == 2) {
    num_extra_cars = atoi(argv[1]);
    if (num_extra_cars < 0) {
      num_extra_cars = 0;
    }
  }

  // Make a linear system to map NPC car state to the state vector of a
  // floating joint, allowing motion and steering in the x-y plane only.
  //
  Drake::XyzRpy<double> y0(0., 0., 0., 0., 0., M_PI / 2.);

  const int insize = Drake::TrivialCarState<double>::RowsAtCompileTime;
  const int outsize = Drake::XyzRpy<double>::RowsAtCompileTime;
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
                        Drake::TrivialCarState,
                        Drake::XyzRpy> >(
      Eigen::MatrixXd::Zero(0, 0),
      Eigen::MatrixXd::Zero(0, insize),
      Eigen::VectorXd::Zero(0),
      Eigen::MatrixXd::Zero(outsize, 0), D,
      toEigen(y0));

  const std::string SEDAN_URDF = Drake::getDrakePath() +
      "/examples/MultiCar/sedan.urdf";
  const std::string BREADTRUCK_URDF = Drake::getDrakePath() +
      "/examples/MultiCar/breadtruck.urdf";

  // RigidBodyTree for visualization.
  auto world_tree = std::make_shared<RigidBodyTree>();

  // NarySystem for car 'physics'.
  // NB:  One could compose the other way as well.
  auto cars_system = std::make_shared<Drake::NArySystem<Drake::TrivialCar> >();
  // NarySystem for car visualization.
  auto cars_vis_adapter = std::make_shared<Drake::NArySystem<decltype(car_vis_adapter)::element_type> >();

  // Create one Sedan.
  world_tree->addRobotFromURDF(SEDAN_URDF,
                               DrakeJoint::ROLLPITCHYAW,
                               nullptr /*weld_to_frame*/);
  // TODO maddog  Hmm... it appears that drake_visualizer wants unique names,
  //              on *links*, otherwise only one of the same-named links will
  //              be get updated joint parameters.
  world_tree->bodies.back()->linkname = "sedan1";
  auto sedan_system = std::make_shared<Drake::TrivialCar>(0., 5., 0.3, 2.0);
  cars_system->addSystem(sedan_system);
  cars_vis_adapter->addSystem(car_vis_adapter);

  // Create one Breadtruck.
  world_tree->addRobotFromURDF(BREADTRUCK_URDF,
                               DrakeJoint::ROLLPITCHYAW,
                               nullptr /*weld_to_frame*/);
  world_tree->bodies.back()->linkname = "breadtruck1";
  auto breadtruck_system = std::make_shared<Drake::TrivialCar>(15., 0., 0.7, 10.0);
  cars_system->addSystem(breadtruck_system);
  cars_vis_adapter->addSystem(car_vis_adapter);

  // Create another Sedan.
  world_tree->addRobotFromURDF(SEDAN_URDF,
                               DrakeJoint::ROLLPITCHYAW,
                               nullptr /*weld_to_frame*/);
  world_tree->bodies.back()->linkname = "sedan2";
  auto sedan2_system = std::make_shared<Drake::TrivialCar>(-5., 0., -0.3, -2.0);
  cars_system->addSystem(sedan2_system);
  cars_vis_adapter->addSystem(car_vis_adapter);

  // Meh, need more cars!
  for (int i = 0; i < num_extra_cars; ++i) {
    world_tree->addRobotFromURDF(
        (i % 2) ? SEDAN_URDF : BREADTRUCK_URDF,
        DrakeJoint::ROLLPITCHYAW,
        nullptr /*weld_to_frame*/);
    std::ostringstream name;
    name << "car-" << i;
    world_tree->bodies.back()->linkname = name.str();
    auto system = std::make_shared<Drake::TrivialCar>(
        (i % 19 - 10) * 10.,
        ((i + 11) % 23 - 11) * 10.,
        0.0,
        ((i % 10) - 5) * 2.0);
    cars_system->addSystem(system);
    cars_vis_adapter->addSystem(car_vis_adapter);
  }


  // NpcCarSystem:
  //  U:  ()
  //  X:  ()
  //  Y:  (xy-position, heading)   SingleNpcCarState

  // BotVisualizer:  DrakeJoint::ROLLPITCHYAW per NPC
  //  U: (x, y, z, roll, pitch, yaw)
  //  X: ()
  //  Y: [== U]

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
