
#include <iostream>

#include "atlas_system.h"
#include "drake/systems/plants/BotVisualizer.h"

using namespace std;
using namespace Drake;
using namespace Eigen;

using drake::AtlasSystem;

int main(int argc, char* argv[]) {
  SimulationOptions options;

  // Get the final time of the simulation.
  double final_time =
      argc >= 2 ? atof(argv[1]) : std::numeric_limits<double>::infinity();

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  if (!lcm->good()) return 1;

  std::shared_ptr<AtlasSystem> atlas_sys = std::make_shared<AtlasSystem>();

  auto const& tree = atlas_sys->getRigidBodyTree();

  options.initial_step_size = 5e-5;
  options.realtime_factor = 0.0;

  auto visualizer =
      make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  auto sys_with_vis = cascade(atlas_sys, visualizer);

  runLCM(sys_with_vis, lcm, 0, final_time, atlas_sys->get_initial_state(),
         options);
}
