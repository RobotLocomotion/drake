#include "drake/automotive/maliput/utility/derive_lane_s_route.h"

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane_data.h"

namespace drake {
namespace maliput {
namespace utility {
namespace {

using api::Lane;
using api::LaneEnd;
using api::LaneEndSet;

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

}  // namespace

std::vector<std::vector<const Lane*>> FindLaneSequences(const Lane& start,
                                                        const Lane& end) {
  if (start.id() == end.id()) {
    return {{&start}};
  }
  return FindLaneSequencesHelper(start, end, {&start}, 0);
}

}  // namespace utility
}  // namespace maliput
}  // namespace drake
