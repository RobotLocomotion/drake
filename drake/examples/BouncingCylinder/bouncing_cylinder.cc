
#include <iostream>

#include <bot_lcmgl_client/lcmgl.h>

#include "cylinder_system.h"
#include "drake/systems/plants/BotVisualizer.h"

using namespace std;
using namespace Drake;
using namespace Eigen;

using drake::CylinderSystem;

int main(int argc, char* argv[]) {
  // Get the final time of the simulation.
  double final_time =
      argc >= 2 ? atof(argv[1]) : std::numeric_limits<double>::infinity();

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  if (!lcm->good()) return 1;

  bot_lcmgl_t* lcmgl = bot_lcmgl_init(lcm->getUnderlyingLCM(), "Collision Info");

  // Unfortunately Drake::cascade takes std::shared_ptr's as arguments.
  std::shared_ptr<CylinderSystem> sys = std::make_shared<CylinderSystem>();
  sys->lcmgl_ = lcmgl;
  //sys->use_multi_contact = true;

  auto const& tree = sys->getRigidBodyTree();

  SimulationOptions options = Drake::default_simulation_options;
  options.initial_step_size = 5e-3;
  options.realtime_factor = 0.0;

  auto visualizer =
      make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  auto sys_with_vis = cascade(sys, visualizer);

  runLCM(sys_with_vis, lcm, 0, final_time, sys->get_initial_state(), options);
  bot_lcmgl_destroy(lcmgl);
}
