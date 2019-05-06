#include "drake/automotive/maliput/utility/generate_string.h"

#include <sstream>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/segment.h"

namespace drake {
namespace maliput {
namespace utility {

std::string GenerateString(const api::RoadGeometry& road_geometry,
                           const GenerateStringOptions& options) {
  std::stringstream result;
  if (options.include_road_geometry_id) {
    result << road_geometry.id().string() << "\n";
  }
  const std::string junction_prefix =
      ((options.include_junction_ids && options.include_road_geometry_id) ? "  "
                                                                          : "");
  for (int ji = 0; ji < road_geometry.num_junctions(); ++ji) {
    const api::Junction* junction = road_geometry.junction(ji);
    if (options.include_junction_ids) {
      result << junction_prefix << junction->id().string() << "\n";
    }
    const std::string segment_prefix =
        junction_prefix +
        ((options.include_segment_ids &&
          (options.include_road_geometry_id || options.include_junction_ids))
             ? "  "
             : "");
    for (int si = 0; si < junction->num_segments(); ++si) {
      const api::Segment* segment = junction->segment(si);
      if (options.include_segment_ids) {
        result << segment_prefix << segment->id().string() << "\n";
      }
      if (options.include_lane_ids) {
        const std::string lane_prefix =
            segment_prefix +
            ((options.include_road_geometry_id ||
              options.include_junction_ids || options.include_segment_ids)
                 ? "  "
                 : "");
        for (int li = 0; li < segment->num_lanes(); ++li) {
          const api::Lane* lane = segment->lane(li);
          result << lane_prefix << lane->id().string() << "\n";
          if (options.include_lane_details) {
            result << lane_prefix << "  s_max: " << lane->length() << "\n";
            result << lane_prefix
                   << "  geo positions (with r = 0 and h = 0):\n";
            result << lane_prefix << "    s_min: "
                   << lane->ToGeoPosition(api::LanePosition(0, 0, 0)) << "\n";
            result << lane_prefix << "    s_max: "
                   << lane->ToGeoPosition(
                          api::LanePosition(lane->length(), 0, 0))
                   << "\n";
          }
        }
      }
    }
  }
  const std::string s = result.str();
  return (s.size() > 0 ? s.substr(0, s.size() - 1) : s);
}

}  // namespace utility
}  // namespace maliput
}  // namespace drake
