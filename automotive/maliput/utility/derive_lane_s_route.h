#pragma once

#include <vector>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/rules/regions.h"

namespace drake {
namespace maliput {
namespace utility {

/// Finds and returns sequences of lanes that go from a specified start lane to
/// a specified end lane. All adjacent and ongoing lanes in both sides of a lane
/// are searched. If @p start and @p end are the same lane, only one sequence of
/// length one is returned.
///
/// @param start The lane at the start of the sequence.
/// @param end The lane at the end of the sequence.
/// @return A vector of sequences of lanes. A vector of length zero is returned
/// if no sequences are found.
std::vector<std::vector<const api::Lane*>> FindLaneSequences(
    const api::Lane& start, const api::Lane& end);

/// Derives and returns a set of LaneSRoute objects that go from @p start to
/// @p end. If no routes are found, a vector of length zero is returned.
std::vector<api::rules::LaneSRoute> DeriveLaneSRoutes(
    const api::RoadPosition& start, const api::RoadPosition& end);

}  // namespace utility
}  // namespace maliput
}  // namespace drake
