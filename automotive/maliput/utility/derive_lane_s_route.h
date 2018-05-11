#pragma once

#include <vector>

#include "drake/automotive/maliput/api/lane.h"

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

}  // namespace utility
}  // namespace maliput
}  // namespace drake
