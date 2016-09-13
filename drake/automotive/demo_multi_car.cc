#include <cmath>
#include <cstdlib>
#include <memory>
#include <sstream>

#include "drake/common/drake_path.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/n_ary_state.h"
#include "drake/systems/n_ary_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/RigidBodyTree.h"

#include "drake/automotive/car_simulation.h"
#include "drake/automotive/trajectory_car.h"

namespace drake {
namespace cars {
namespace {

using drake::systems::plants::joints::kRollPitchYaw;

int DoMain(int argc, const char* argv[]) {
  int num_cars = 100;
  if (argc == 2) {
    num_cars = atoi(argv[1]);
    if (num_cars < 1) {
      std::cerr << "The number of cars must be >= 1.\n";
      std::exit(1);
    }
  }

  auto car_vis_adapter = CreateSimpleCarVisualizationAdapter();

  const std::string kSedanUrdf = drake::GetDrakePath() +
      "/automotive/models/sedan.urdf";
  const std::string kBreadtruckUrdf = drake::GetDrakePath() +
      "/automotive/models/breadtruck.urdf";

  // RigidBodyTree for visualization.
  auto world_tree = std::make_shared<RigidBodyTree>();

  // NarySystem for car 'physics'.
  //  U: ()
  //  X: ()
  //  Y: [(xy-position, heading, velocity), ...] per SimpleCarState
  auto cars_system = std::make_shared<NArySystem<TrajectoryCar1>>();
  // NarySystem for car visualization.
  // BotVisualizer:
  //  U: [(xy-position, heading, velocity), ...] per SimpleCarState
  //  X: ()
  //  Y: [(x, y, z, roll, pitch, yaw), ...] per kRollPitchYaw per car
  auto cars_vis_adapter = std::make_shared<
    NArySystem<decltype(car_vis_adapter)::element_type>>();
  // NB:  One could compose the other way as well (i.e., individually cascade
  // each TrajectoryCar with a car_vis_adapter, and then stack each of those
  // pairs into a single NArySystem).

  // Add all of the desired cars.
  for (int i = 0; i < num_cars; ++i) {
    // Add the visualization entity.
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        (i % 5) ? kSedanUrdf : kBreadtruckUrdf, kRollPitchYaw,
        nullptr /* weld_to_frame */, world_tree.get());

    // Add the trajectory car, and its visualization adapter.
    cars_system->AddSystem(CreateTrajectoryCarSystem(i));
    cars_vis_adapter->AddSystem(car_vis_adapter);
  }

  // Add LCM support, for visualization.
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
  auto visualizer =
      std::make_shared<drake::BotVisualizer<
        decltype(cars_vis_adapter)::element_type::OutputVector>>(
            lcm, world_tree);

  // Compose the system together from the parts.
  auto the_system = drake::cascade(drake::cascade(
      cars_system, cars_vis_adapter), visualizer);

  decltype(the_system)::element_type::StateVector<double> initial_state;
  drake::SimulationOptions options;
  options.realtime_factor = 0.0;
  const double t0 = 0.0;
  const double tf = std::numeric_limits<double>::infinity();
  runLCM(the_system, lcm, t0, tf, initial_state, options);

  return 0;
}

}  // namespace
}  // namespace cars
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::cars::DoMain(argc, argv);
}
