#pragma once

#include <memory>
#include <string>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/base/simple_rulebook.h"

namespace drake {
namespace maliput {

/// Instantiates RightOfWayRule instances based on the specified @p
/// road_geometry and @p input document.
///
/// @param road_geometry The road geometry to which the loaded RightOfWayRule
/// instances apply.
///
/// @param input The YAML RightOfWayRules document.
///
/// @return A SimpleRulebook containing the loaded RightOfWayRule instances.
///
/// @throws std::exception if the YAML document within @p input is invalid, or a
/// RightOfWayRule within the YAML document has an invalid state or zone type.
std::unique_ptr<SimpleRulebook> LoadRightOfWayRules(
    const api::RoadGeometry* road_geometry, const std::string& input);

/// Instantiates RightOfWayRule instances based on the specified @p
/// road_geometry and file.
///
/// @param road_geometry The road geometry to which the loaded RightOfWayRule
/// instances apply.
///
/// @param filename The YAML file that contains an RightOfWayRules document.
///
/// @return A SimpleRulebook containing the loaded RightOfWayRule instances.
///
/// @throws std::exception if @p filename cannot be found, the YAML document
/// within @p filename is invalid, or a RightOfWayRule within the YAML document
/// has an invalid state or zone type.
std::unique_ptr<SimpleRulebook> LoadRightOfWayRulesFromFile(
    const api::RoadGeometry* road_geometry, const std::string& filename);

}  // namespace maliput
}  // namespace drake
