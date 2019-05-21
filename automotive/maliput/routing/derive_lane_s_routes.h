#pragma once

#include <vector>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/rules/regions.h"

namespace drake {
namespace maliput {
namespace routing {

/// Derives and returns a set of LaneSRoute objects that go from @p start to
/// @p end. If no routes are found, a vector of length zero is returned.
/// Parameter @p max_length_m is the maximum length of the intermediate lanes
/// between @p start and @p end. See the description of FindLaneSequences() for
/// more details. If @p start and @p end are the same lane, a route consisting
/// of one lane is returned regardless of @p max_length_m.
std::vector<api::rules::LaneSRoute> DeriveLaneSRoutes(
    const api::RoadPosition& start, const api::RoadPosition& end,
    double max_length_m);

}  // namespace routing
}  // namespace maliput
}  // namespace drake
