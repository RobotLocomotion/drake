#pragma once

#include <string>

#include "drake/automotive/maliput/api/road_geometry.h"

namespace drake {
namespace maliput {
namespace utility {

/// Parameters that specify what details about an api::RoadGeometry to print.
struct GenerateStringOptions {
  bool include_road_geometry_id{false};
  bool include_junction_ids{false};
  bool include_segment_ids{false};
  bool include_lane_ids{false};
  /// Whether to include lane details like s_min, s_max, and the geo positions
  /// at (s_min, 0, 0) and (s_max, 0, 0).
  bool include_lane_details{false};
};

/// Generates and returns a string containing details about the provided
/// api::RoadGeometry.
///
/// @param road_geometry The api::RoadGeometry.
/// @param options Options that affect the types of information to include in
/// the returned string.
/// @return The generated string.
std::string GenerateString(const api::RoadGeometry& road_geometry,
                           const GenerateStringOptions& options);

}  // namespace utility
}  // namespace maliput
}  // namespace drake
