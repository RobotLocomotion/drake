// This file is licensed under the MIT-0 License.
// See LICENSE-MIT-0.txt in the current directory.

#pragma once

#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <variant>
#include <vector>

#include "drake/common/name_value.h"
#include "drake/geometry/scene_graph_config.h"
#include "drake/lcm/drake_lcm_params.h"
#include "drake/manipulation/kuka_iiwa/iiwa_driver.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_driver.h"
#include "drake/manipulation/util/named_positions_functions.h"
#include "drake/manipulation/util/zero_force_driver.h"
#include "drake/multibody/parsing/model_directives.h"
#include "drake/multibody/plant/multibody_plant_config.h"
#include "drake/systems/analysis/simulator_config.h"
#include "drake/systems/sensors/camera_config.h"
#include "drake/visualization/visualization_config.h"

namespace drake {
namespace internal {

/* Defines the YAML format for a (possibly stochastic) scenario to be
simulated.

Drake maintainers should keep this file in sync with hardware_sim.py. */
struct Scenario {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(random_seed));
    a->Visit(DRAKE_NVP(simulation_duration));
    a->Visit(DRAKE_NVP(simulator_config));
    a->Visit(DRAKE_NVP(plant_config));
    a->Visit(DRAKE_NVP(scene_graph_config));
    a->Visit(DRAKE_NVP(directives));
    a->Visit(DRAKE_NVP(lcm_buses));
    a->Visit(DRAKE_NVP(model_drivers));
    a->Visit(DRAKE_NVP(cameras));
    a->Visit(DRAKE_NVP(visualization));
    a->Visit(DRAKE_NVP(initial_position));
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

  /* Plant configuration (time step and contact parameters). */
  multibody::MultibodyPlantConfig plant_config;

  /* SceneGraph configuration. */
  geometry::SceneGraphConfig scene_graph_config;

  /* All of the fully deterministic elements of the simulation. */
  std::vector<multibody::parsing::ModelDirective> directives;

  /* A map of {bus_name: lcm_params} for LCM transceivers to be used by drivers,
  sensors, etc. */
  std::map<std::string, lcm::DrakeLcmParams> lcm_buses{{"default", {}}};

  /* For actuated models, specifies where each model's actuation inputs come
  from, keyed on the ModelInstance name. */
  using DriverVariant = std::variant<manipulation::kuka_iiwa::IiwaDriver,
                                     manipulation::schunk_wsg::SchunkWsgDriver,
                                     manipulation::ZeroForceDriver>;
  std::map<std::string, DriverVariant> model_drivers;

  /* Cameras to add to the scene (and broadcast over LCM). The key for each
   camera is a helpful mnemonic, but does not serve a technical role. The
   CameraConfig::name field is still the name that will appear in the Diagram
   artifacts. */
  std::map<std::string, systems::sensors::CameraConfig> cameras;

  visualization::VisualizationConfig visualization;

  /* Optional initial positions to override or supplement those in the
  directives. */
  manipulation::NamedPositions initial_position;
};

/* Returns a C++ representation of the given YAML scenario.
Refer to the gflags descriptions atop hardware_sim.cc for details. */
Scenario LoadScenario(const std::string& filename,
                      const std::string& scenario_name,
                      const std::string& scenario_text = "{}");

/* Returns a YAML representation of this C++ scenario.
@param verbose When saving the scenario, whether or not default values should be
written out to be explicit (true) or omitted (false). Verbosity helps defend
logged testset scenarios against changing defaults. */
std::string SaveScenario(const Scenario& scenario,
                         const std::string& scenario_name,
                         bool verbose = false);

}  // namespace internal
}  // namespace drake
