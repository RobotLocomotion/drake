#pragma once

#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "drake/common/name_value.h"
#include "drake/lcm/drake_lcm_params.h"
#include "drake/multibody/parsing/model_directives.h"
#include "drake/multibody/plant/multibody_plant_config.h"
#include "drake/systems/analysis/simulator_config.h"
#include "drake/visualization/visualization_config.h"

namespace drake {

/* Defines the YAML format for a (possibly stochastic) scenario to be
simulated. */
struct Scenario {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(random_seed));
    a->Visit(DRAKE_NVP(simulation_duration));
    a->Visit(DRAKE_NVP(simulator_config));
    a->Visit(DRAKE_NVP(plant_config));
    a->Visit(DRAKE_NVP(directives));
    a->Visit(DRAKE_NVP(lcm_buses));
    a->Visit(DRAKE_NVP(visualization));
  }

  /* Random seed for any random elements in the scenario.
  The seed is always deterministic in the `Scenario`; a caller who wants
  randomness must populate this value from their own randomness. */
  std::uint64_t random_seed{0};

  /* The maximum simulation time (in seconds).  The simulator will attempt to
  run until this time and then terminate. */
  double simulation_duration{std::numeric_limits<double>::infinity()};

  /* Simulator configuration (integrator and publisher parameters). */
  systems::SimulatorConfig simulator_config{
    .max_step_size = 1e-3,
    .accuracy = 1.0e-2,
    .target_realtime_rate = 1.0,
  };

  /* Plant configuration (timestep and contact parameters). */
  multibody::MultibodyPlantConfig plant_config;

  /* All of the fully deterministic elements of the simulation. */
  std::vector<multibody::parsing::ModelDirective> directives;

  /* A map of {bus_name: lcm_params} for LCM transceivers to be used by drivers,
  sensors, etc. */
  std::map<std::string, lcm::DrakeLcmParams> lcm_buses{{"default", {}}};

  visualization::VisualizationConfig visualization;
};

/* Returns a C++ representation of the given YAML scenario.
Refer to the gflags descriptions atop hardware_sim.cc for details. */
Scenario LoadScenario(
    const std::string& filename,
    const std::string& scenario_name,
    const std::string& scenario_text = "{}");

/* Returns a YAML representation of this C++ scenario.
@param verbose When saving the scenario, whether or not default values should be
written out to be explicit (true) or omitted (false). Verbosity helps defend
logged testset scenarios against changing defaults. */
std::string SaveScenario(
    const Scenario& scenario,
    const std::string& scenario_name,
    bool verbose = false);

}  // namespace drake
