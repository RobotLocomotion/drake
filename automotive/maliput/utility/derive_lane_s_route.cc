#include "drake/automotive/maliput/utility/derive_lane_s_route.h"

#include <sstream>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace utility {
namespace {

using api::Lane;
using api::LaneEnd;
using api::LaneEndSet;
using api::LanePosition;
using api::rules::LaneSRange;
using api::rules::LaneSRoute;
using api::rules::SRange;

// Defines the maximum sequence length. This prevents memory overflows when
// dealing with giant road networks.
int constexpr kMaxSequenceLength{100};

void AddLanesInLaneEndSet(const LaneEndSet* lane_end_set,
                          std::vector<const Lane*>* container) {
  for (int i = 0; i < lane_end_set->size(); ++i) {
    const LaneEnd& lane_end = lane_end_set->get(i);
    container->push_back(lane_end.lane);
  }
}

std::vector<std::vector<const Lane*>> FindLaneSequencesHelper(const Lane& start,
    const Lane& end, const std::vector<const Lane*>& visited_list,
    int current_length) {
  std::vector<std::vector<const Lane*>> result;
  if (current_length >= kMaxSequenceLength) return result;

  // Gather all of the start lane's ongoing and neighboring lanes. These are the
  // lanes that will be checked in search for the end lane.
  std::vector<const Lane*> lanes_to_check;
  AddLanesInLaneEndSet(start.GetOngoingBranches(api::LaneEnd::kStart),
                       &lanes_to_check);
  AddLanesInLaneEndSet(start.GetOngoingBranches(api::LaneEnd::kFinish),
                       &lanes_to_check);
  if (start.to_left()) lanes_to_check.push_back(start.to_left());
  if (start.to_right()) lanes_to_check.push_back(start.to_right());

  for (const auto& next_lane : lanes_to_check) {
    if (std::find(visited_list.begin(), visited_list.end(), next_lane)
        != visited_list.end()) {
      continue;
    }
    if (next_lane->id() == end.id()) {
      result.push_back({&start, &end});
    } else {
      std::vector<const Lane*> new_visited_list = visited_list;
      new_visited_list.push_back(next_lane);
      const auto subsequences = FindLaneSequencesHelper(
          *next_lane, end, new_visited_list, current_length + 1);
      for (std::vector<const Lane*> sequence : subsequences) {
        sequence.insert(sequence.begin(), &start);
        result.push_back(sequence);
      }
    }
  }
  return result;
}

// Returns true if @p lane is in @p set and false otherwise.
bool LaneExistsInSet(const LaneEndSet* set, const Lane* lane) {
  DRAKE_ASSERT(set != nullptr);
  for (int i = 0; i < set->size(); ++i) {
    if (set->get(i).lane == lane) {
      return true;
    }
  }
  return false;
}

// Returns the S coordinate in @p lane that is on the border with
// @p next_lane.
optional<double> DetermineEdgeS(const Lane* lane, const Lane* next_lane) {
  DRAKE_ASSERT(lane != nullptr);
  DRAKE_ASSERT(next_lane != nullptr);
  if (LaneExistsInSet(lane->GetOngoingBranches(LaneEnd::kFinish), next_lane)) {
    return lane->length();
  }
  if (LaneExistsInSet(lane->GetOngoingBranches(LaneEnd::kStart), next_lane)) {
    return 0;
  }
  return nullopt;
}

}  // namespace

std::vector<std::vector<const Lane*>> FindLaneSequences(const Lane& start,
                                                        const Lane& end) {
  if (start.id() == end.id()) {
    return {{&start}};
  }
  return FindLaneSequencesHelper(start, end, {&start}, 0);
}

std::vector<LaneSRoute> DeriveLaneSRoutes(const api::RoadPosition& start,
                                          const api::RoadPosition& end) {
  DRAKE_ASSERT(start.lane != nullptr);
  DRAKE_ASSERT(end.lane != nullptr);
  const double start_s = start.pos.s();
  const double end_s = end.pos.s();
  std::vector<LaneSRoute> result;

  for (const auto& lane_sequence : FindLaneSequences(*start.lane, *end.lane)) {
    DRAKE_ASSERT(lane_sequence.size() > 0);
    std::vector<LaneSRange> ranges;

    // Handles the case when lane_sequence has a length of 1. This occurs when
    // start and end are in the same lane.
    if (lane_sequence.size() == 1) {
      DRAKE_ASSERT(start.lane == end.lane);
      const std::vector<LaneSRange> lane_s_ranges =
          {LaneSRange(start.lane->id(), SRange(start_s, end_s))};
      result.emplace_back(lane_s_ranges);
      continue;
    }

    // Handles the case when lane_sequence has a length greater than 1.
    bool is_s_continuous{true};
    for (size_t i = 0; is_s_continuous && i < lane_sequence.size(); ++i) {
      const Lane* lane = lane_sequence.at(i);
      DRAKE_ASSERT(lane != nullptr);
      if (lane->id() == start.lane->id()) {
        DRAKE_ASSERT(i == 0);
        DRAKE_ASSERT(lane_sequence.size() > 1);
        const optional<double> first_end_s =
            DetermineEdgeS(lane, lane_sequence.at(1));
        if (!first_end_s) {
          is_s_continuous = false;
          continue;
        }
        ranges.push_back(
            LaneSRange(lane->id(), SRange(start_s, first_end_s.value())));
      } else if (lane->id() == end.lane->id()) {
        DRAKE_ASSERT(i == lane_sequence.size() - 1);
        DRAKE_ASSERT(i > 0);
        const optional<double> last_start_s =
            DetermineEdgeS(lane, lane_sequence.at(i - 1));
        if (!last_start_s) {
          is_s_continuous = false;
          continue;
        }
        ranges.push_back(
            LaneSRange(lane->id(), SRange(last_start_s.value(), end_s)));
      } else {
        const optional<double> middle_start_s =
            DetermineEdgeS(lane, lane_sequence.at(i - 1));
        const optional<double> middle_end_s =
            DetermineEdgeS(lane, lane_sequence.at(i + 1));
        if (!middle_start_s || !middle_end_s) {
          is_s_continuous = false;
          continue;
        }
        ranges.push_back(
            LaneSRange(lane->id(), SRange(middle_start_s.value(),
                                          middle_end_s.value())));
      }
    }
    if (is_s_continuous) {
      result.emplace_back(ranges);
    }
  }
  return result;
}

}  // namespace utility
}  // namespace maliput
}  // namespace drake
