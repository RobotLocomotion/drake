#pragma once

#include <memory>
#include <string>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/rules/road_rulebook.h"

namespace drake {
namespace maliput {

/// Instantiates an api::rules::RoadRulebook based on the specified @p
/// road_geometry and @p input.
///
/// @param road_geometry The road geometry to which the api::rules::RoadRulebook
/// to be loaded applies.
///
/// @param input The YAML RoadRulebook document.
///
/// @return An api::rules::RoadRulebook containing the rules specified in
/// @p input.
///
/// @throws std::exception if the YAML document within @p input is invalid, or
/// an api::rules::RightOfWayRule within @p input has an invalid state or zone
/// type.
std::unique_ptr<api::rules::RoadRulebook> LoadRoadRulebook(
    const api::RoadGeometry* road_geometry, const std::string& input);

/// Instantiates an api::rules::RoadRulebook based on the specified @p
/// road_geometry and @p filename.
///
/// @param road_geometry The road geometry to which the api::rules::RoadRulebook
/// to be loaded applies.
///
/// @param input The YAML RoadRulebook document.
///
/// @return An api::rules::RoadRulebook containing the rules specified in
/// @p input.
///
/// @throws std::exception if the YAML document within @p input is invalid, or
/// an api::rules::RightOfWayRule within @p input has an invalid state or zone
/// type.
std::unique_ptr<api::rules::RoadRulebook> LoadRoadRulebookFromFile(
    const api::RoadGeometry* road_geometry, const std::string& filename);

}  // namespace maliput
}  // namespace drake
