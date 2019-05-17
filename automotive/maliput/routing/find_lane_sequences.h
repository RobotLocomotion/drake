#pragma once

#include <vector>

#include "drake/automotive/maliput/api/lane.h"

namespace drake {
namespace maliput {
namespace routing {

/// Finds and returns sequences of lanes that go from a specified start lane to
/// a specified end lane. Only ongoing lanes are searched (adjacent lanes are
/// not). If @p start and @p end are the same lane, a sequence of one lane is
/// returned regardless of @p max_length_m.
///
/// @param start The lane at the start of the sequence.
/// @param end The lane at the end of the sequence.
/// @param max_length_m The maximum length of a sequence in meters, not
/// including @p start and @p end. The lengths of @p start and @p end are not
/// included because a vehicle may not fully traverse them. Getting from
/// @p start to @p end, however, requires a vehicle to fully traverse all
/// intermediate lanes in the sequence, which is why only the sum of their
/// lengths are included in the comparison with this upper bound.
/// @return A vector of lane sequences in which the first lane is @p start and
/// the last lane is @p end. An empty vector is returned if no sequences are
/// found.
std::vector<std::vector<const drake::maliput::api::Lane*>> FindLaneSequences(
    const drake::maliput::api::Lane* start,
    const drake::maliput::api::Lane* end, double max_length_m);

}  // namespace routing
}  // namespace maliput
}  // namespace drake
