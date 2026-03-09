// This file is licensed under the MIT-0 License.
// See LICENSE-MIT-0.txt in the current directory.

#include "drake/examples/hardware_sim/scenario.h"

#include <utility>

#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace internal {

// N.B. This is in a cc file to reduce compilation time for the serialization-
// related template classes and functions.
Scenario LoadScenario(const std::string& filename,
                      const std::string& scenario_name,
                      const std::string& scenario_text) {
  // Begin with the constructor defaults.
  Scenario scenario;

  // Add in changes from the scenario file (if given).
  if (!filename.empty()) {
    scenario = drake::yaml::LoadYamlFile<Scenario>(filename, {scenario_name},
                                                   {std::move(scenario)});
  }

  // Add in changes from the extra scenario text.
  scenario = drake::yaml::LoadYamlString<Scenario>(
      scenario_text, std::nullopt /* no name */, {std::move(scenario)});

  return scenario;
}

// N.B. This is in a cc file to reduce compilation time for the serialization-
// related template classes and functions.
std::string SaveScenario(const Scenario& scenario,
                         const std::string& scenario_name, bool verbose) {
  const auto defaults = verbose ? std::nullopt : std::optional{Scenario{}};
  return drake::yaml::SaveYamlString(scenario, {scenario_name}, defaults);
}

}  // namespace internal
}  // namespace drake
