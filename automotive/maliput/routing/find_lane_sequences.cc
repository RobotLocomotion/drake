#include "drake/automotive/maliput/routing/find_lane_sequences.h"

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane_data.h"

using drake::maliput::api::Lane;
using drake::maliput::api::LaneEnd;
using drake::maliput::api::LaneEndSet;

namespace drake {
namespace maliput {
namespace routing {
namespace {

// Adds the lanes in @p lane_end_set to @p container.
void AddLanes(const LaneEndSet* lane_end_set,
              std::vector<const Lane*>* container) {
  for (int i = 0; i < lane_end_set->size(); ++i) {
    const LaneEnd& lane_end = lane_end_set->get(i);
    container->push_back(lane_end.lane);
  }
}

std::vector<std::vector<const Lane*>> FindLaneSequencesHelper(
    const Lane* start, const Lane* end,
    const std::vector<const Lane*>& visited_list, double max_length_m,
    double current_length_m) {
  std::vector<std::vector<const Lane*>> result;
  if (current_length_m > max_length_m) return result;

  // Gather all of the start lane's ongoing lanes. These are the lanes that will
  // be checked in search for the end lane.
  std::vector<const Lane*> lanes_to_check;
  AddLanes(start->GetOngoingBranches(LaneEnd::kStart), &lanes_to_check);
  AddLanes(start->GetOngoingBranches(LaneEnd::kFinish), &lanes_to_check);

  for (const auto& next_lane : lanes_to_check) {
    if (std::find(visited_list.begin(), visited_list.end(), next_lane) !=
        visited_list.end()) {
      continue;
    }
    if (next_lane->id() == end->id()) {
      result.push_back({start, end});
    } else {
      std::vector<const Lane*> new_visited_list = visited_list;
      new_visited_list.push_back(next_lane);
      const auto subsequences = FindLaneSequencesHelper(
          next_lane, end, new_visited_list, max_length_m,
          current_length_m + next_lane->length());
      for (std::vector<const Lane*> subsequence : subsequences) {
        subsequence.insert(subsequence.begin(), start);
        result.push_back(subsequence);
      }
    }
  }
  return result;
}

}  // namespace

std::vector<std::vector<const Lane*>> FindLaneSequences(const Lane* start,
                                                        const Lane* end,
                                                        double max_length_m) {
  if (start->id() == end->id()) {
    return {{start}};
  }
  return FindLaneSequencesHelper(start, end, {start}, max_length_m, 0);
}

}  // namespace routing
}  // namespace maliput
}  // namespace drake
