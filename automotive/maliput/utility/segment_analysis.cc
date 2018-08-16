#include "drake/automotive/maliput/utility/segment_analysis.h"

#include <queue>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"

namespace drake {
namespace maliput {
namespace utility {

std::unordered_set<const api::Segment*>
FindConfluentSegments(const api::Segment* const seed_segment) {
  // Make an empty workqueue (FIFO).
  std::queue<const api::Segment*> workqueue;
  // Make an empty visited set.
  std::unordered_set<const api::Segment*> visited;

  // Add seed_segment to the workqueue.
  workqueue.push(seed_segment);

  // Work the queue.
  while (!workqueue.empty()) {
    const api::Segment* working_segment = workqueue.front();
    workqueue.pop();

    // Loop over each Lane in the Segment.
    for (int lane_index = 0;
         lane_index < working_segment->num_lanes(); ++lane_index) {
      const api::Lane* const lane = working_segment->lane(lane_index);
      // Loop over each End of the Lane.
      for (const api::LaneEnd::Which end :
        {api::LaneEnd::kStart, api::LaneEnd::kFinish}) {
        const api::LaneEndSet* const confluent_set =
            lane->GetConfluentBranches(end);
        // Loop over the set of confluent lanes.
        for (int i = 0; i < confluent_set->size(); ++i) {
          // Get confluent segment.
          const api::Segment* confluent_segment =
              confluent_set->get(i).lane->segment();
          // Is confluent segment in visited set?
          if (visited.find(confluent_segment) == visited.end()) {
            // Not yet:  then push onto end of workqueue, and mark as visited.
            workqueue.push(confluent_segment);
            visited.insert(confluent_segment);
          }
        }
      }
    }
  }
  // Return the visited set, which is the set of all Segments connected to
  // seed_segment (including seed_segment itself).
  return visited;
}


std::vector<std::unordered_set<const api::Segment*>>
AnalyzeConfluentSegments(const api::RoadGeometry* road_geometry) {
  std::vector<std::unordered_set<const api::Segment*>> components;
  std::unordered_set<const api::Segment*> visited;
  for (int junction_index = 0;
       junction_index < road_geometry->num_junctions(); ++junction_index) {
    const api::Junction* const junction =
        road_geometry->junction(junction_index);
    for (int segment_index = 0;
         segment_index < junction->num_segments(); ++segment_index) {
      const api::Segment* const segment = junction->segment(segment_index);
      // If segment has already been visited, skip it.
      if (visited.count(segment) > 0) { continue; }
      // Explore segment's connected component.
      components.emplace_back(FindConfluentSegments(segment));
      // Record all segments from the new component as visited.
      visited.insert(components.back().begin(), components.back().end());
    }
  }
  // Return the list of connected components.
  return components;
}


}  // namespace utility
}  // namespace maliput
}  // namespace drake
