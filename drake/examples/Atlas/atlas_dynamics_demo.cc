
#include <iostream>

#include "drake/examples/Atlas/atlas_plant.h"
#include "drake/system1/LCMSystem.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/util/drakeAppUtil.h"

using drake::SimulationOptions;
using drake::BotVisualizer;
using drake::AtlasPlant;

int main(int argc, char* argv[]) {
  try {
    if (argc != 1 && argc != 3) {
      std::cerr << "Usage: " << argv[0] << " [options]" << std::endl
                << "Options: " << std::endl
                << "  --duration [duration in seconds]" << std::endl;
      return 0;
    }

    // Parse command line options.
    double final_time = std::numeric_limits<double>::infinity();
    try {
      char* duration_option =
          getCommandLineOption(argv, argv + argc, "--duration");
      if (duration_option) {
        final_time = std::stod(std::string(duration_option));
      }
    } catch (std::exception& e) {
      std::cout << "Error when parsing command line options: " << e.what()
                << std::endl;
      return 1;
    }

    std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
    if (!lcm->good()) return 1;

    std::shared_ptr<AtlasPlant> atlas_sys = std::make_shared<AtlasPlant>();

    auto const& tree = atlas_sys->get_rigid_body_tree();

    auto visualizer =
        std::make_shared<BotVisualizer<AtlasPlant::StateVector>>(lcm, tree);

    auto sys_with_vis = cascade(atlas_sys, visualizer);

    SimulationOptions options;
    options.initial_step_size = 5e-5;
    options.realtime_factor = 0.0;

    runLCM(sys_with_vis, lcm, 0, final_time, atlas_sys->get_initial_state(),
           options);
  } catch (const std::exception& e) {
    std::cout << "Error: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
