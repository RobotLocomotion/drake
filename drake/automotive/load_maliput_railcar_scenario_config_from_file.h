#pragma once

#include <string>

#include "drake/automotive/maliput_railcar_scenario_config.pb.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace automotive {

/**
 * Loads MaliputRailcarScenarioConfig parameters from a file. The format of
 * the config file is defined in `maliput_railcar_scenario_config.proto`.
 *
 * @param path The path to the config file.
 *
 * @param scenario_config The destination where the parameters should be saved.
 *
 * @throws std::runtime_error if the provided file cannot be parsed.
 */
void LoadMaliputRailcarScenarioConfigFromFile(
    const std::string& path, MaliputRailcarScenarioConfig* scenario_config);
}  // namespace automotive
}  // namespace drake
