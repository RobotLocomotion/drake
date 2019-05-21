#include "drake/automotive/maliput/routing/derive_lane_s_routes.h"

#include <boost/assert.hpp>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/routing/find_lane_sequences.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace routing {
namespace {

// Returns true iff @p lane is in @p set.
bool LaneExistsInSet(const api::LaneEndSet* set, const api::Lane* lane) {
  BOOST_ASSERT(set != nullptr);
  for (int i = 0; i < set->size(); ++i) {
    if (set->get(i).lane == lane) {
      return true;
    }
  }
  return false;
}

// Returns the S coordinate in @p lane that is on the border with @p next_lane.
drake::optional<double> DetermineEdgeS(const api::Lane* lane,
                                       const api::Lane* next_lane) {
  BOOST_ASSERT(lane != nullptr);
  BOOST_ASSERT(next_lane != nullptr);
  if (LaneExistsInSet(lane->GetOngoingBranches(api::LaneEnd::kFinish),
                      next_lane)) {
    return lane->length();
  }
  if (LaneExistsInSet(lane->GetOngoingBranches(api::LaneEnd::kStart),
                      next_lane)) {
    return 0.;
  }
  return drake::nullopt;
}

}  // namespace

std::vector<api::rules::LaneSRoute> DeriveLaneSRoutes(
    const api::RoadPosition& start, const api::RoadPosition& end,
    double max_length_m) {
  BOOST_ASSERT(start.lane != nullptr);
  BOOST_ASSERT(end.lane != nullptr);
  const double start_s = start.pos.s();
  const double end_s = end.pos.s();
  std::vector<api::rules::LaneSRoute> result;

  for (const auto& lane_sequence :
       FindLaneSequences(start.lane, end.lane, max_length_m)) {
    BOOST_ASSERT(!lane_sequence.empty());
    std::vector<api::rules::LaneSRange> ranges;

    // Handles the case when lane_sequence has a length of 1. This occurs when
    // start and end are in the same lane.
    if (lane_sequence.size() == 1) {
      BOOST_ASSERT(start.lane == end.lane);
      const std::vector<api::rules::LaneSRange> lane_s_ranges = {
          api::rules::LaneSRange(start.lane->id(),
                                 api::rules::SRange(start_s, end_s))};
      result.emplace_back(lane_s_ranges);
      continue;
    }

    // Handles the case when lane_sequence has a length greater than 1.
    for (size_t i = 0; i < lane_sequence.size(); ++i) {
      const api::Lane* lane = lane_sequence.at(i);
      BOOST_ASSERT(lane != nullptr);
      if (i == 0) {
        BOOST_ASSERT(lane->id() == start.lane->id());
        const drake::optional<double> first_end_s =
            DetermineEdgeS(lane, lane_sequence.at(1));
        BOOST_ASSERT(first_end_s);
        ranges.push_back(api::rules::LaneSRange(
            lane->id(), api::rules::SRange(start_s, first_end_s.value())));
      } else if (i + 1 == lane_sequence.size()) {
        BOOST_ASSERT(lane->id() == end.lane->id());
        BOOST_ASSERT(i > 0);
        const drake::optional<double> last_start_s =
            DetermineEdgeS(lane, lane_sequence.at(i - 1));
        BOOST_ASSERT(last_start_s);
        ranges.push_back(api::rules::LaneSRange(
            lane->id(), api::rules::SRange(last_start_s.value(), end_s)));
      } else {
        const drake::optional<double> middle_start_s =
            DetermineEdgeS(lane, lane_sequence.at(i - 1));
        const drake::optional<double> middle_end_s =
            DetermineEdgeS(lane, lane_sequence.at(i + 1));
        BOOST_ASSERT(middle_start_s && middle_end_s);
        ranges.push_back(api::rules::LaneSRange(
            lane->id(),
            api::rules::SRange(middle_start_s.value(), middle_end_s.value())));
      }
    }
    result.emplace_back(ranges);
  }
  return result;
}
}  // namespace routing
}  // namespace maliput
}  // namespace drake
