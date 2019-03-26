#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/automotive/maliput/api/rules/right_of_way_phase_book.h"
#include "drake/automotive/maliput/api/rules/road_rulebook.h"

namespace drake {
namespace maliput {

/// Instantiates and returns an api::rules::RightOfWayPhaseBook instance based
/// on the specified @p rulebook, and @p input document.
///
/// @param rulebook Contains the rules that apply to @p road_geometry.
///
/// @param input The YAML Intersections document.
///
/// @return The newly created api::rules::RightOfWayPhaseBook instance.
std::unique_ptr<api::rules::RightOfWayPhaseBook> LoadPhaseRingBook(
    const api::rules::RoadRulebook* rulebook, const std::string& input);

/// Instantiates and returns an api::rules::RightOfWayPhaseBook instance based
/// on the specified @p rulebook, and @p filename.
///
/// @param rulebook Contains the rules that apply to @p road_geometry.
///
/// @param filename The YAML file that contains an Intersections document.
///
/// @return The newly created api::rules::RightOfWayPhaseBook instance.
std::unique_ptr<api::rules::RightOfWayPhaseBook> LoadPhaseRingBookFromFile(
    const api::rules::RoadRulebook* rulebook, const std::string& filename);

}  // namespace maliput
}  // namespace drake
