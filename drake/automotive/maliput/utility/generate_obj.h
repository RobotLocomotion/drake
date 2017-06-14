#pragma once

#include <string>
#include <vector>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"

namespace drake {
namespace maliput {
namespace utility {

/// Multitude of parameters for generating an OBJ model of a road surface,
/// with sensible defaults.
struct ObjFeatures {
  /// Maximum distance between rendered vertices, in either s- or r-dimension,
  /// along a lane's surface
  double max_grid_unit{1.0};
  /// Minimum number of vertices, in either s- or r-dimension, along a lane's
  /// surface.
  double min_grid_resolution{5.0};
  /// Draw stripes along lane_bounds() of each lane?
  bool draw_stripes{true};
  /// Draw arrows at start/finish of each lane?
  bool draw_arrows{true};
  /// Draw highlighting swath with lane_bounds() of each lane?
  bool draw_lane_haze{true};
  /// Draw branching at BranchPoints?
  bool draw_branch_points{true};
  /// Draw highlighting of elevation_bounds of each lane?
  bool draw_elevation_bounds{true};
  /// Absolute width of stripes
  double stripe_width{0.25};
  /// Absolute elevation (h) of stripes above road surface
  double stripe_elevation{0.05};
  /// Absolute elevation (h) of arrows above road surface
  double arrow_elevation{0.05};
  /// Absolute elevation (h) of lane-haze above road surface
  double lane_haze_elevation{0.02};
  /// Absolute elevation (h) of branch-points above road surface
  double branch_point_elevation{0.5};
  /// Height of rendered branch-point arrows
  double branch_point_height{0.5};
  /// Origin of OBJ coordinates relative to world-frame
  api::GeoPosition origin{0., 0., 0.};
  /// ID's of specific segments to be highlighted.  (If non-empty, then the
  /// Segments *not* specified on this list will be rendered as grayed-out.)
  std::vector<api::SegmentId> highlighted_segments;
};

/// Generates a Wavefront OBJ model of the road surface of an api::RoadGeometry.
///
/// @param road_geometry  the api::RoadGeometry to model
/// @param dirpath  directory component of the output pathnames
/// @param fileroot  root of the filename component of the output pathnames
/// @param features  parameters for constructing the mesh
///
/// GenerateObjFile actually produces two files:  the first, named
/// [@p dirpath]/[@p fileroot].obj, is a Wavefront OBJ containing the
/// mesh which models the api::RoadGeometry.  The second file is a
/// Wavefront MTL file named [@p dirpath]/[@p fileroot].mtl, containing
/// descriptions of materials referenced by the OBJ file.
///
/// The produced mesh covers the area within the driveable-bounds of the
/// road surface described by the RoadGeometry.
void GenerateObjFile(const api::RoadGeometry* road_geometry,
                     const std::string& dirpath,
                     const std::string& fileroot,
                     const ObjFeatures& features);

}  // namespace utility
}  // namespace maliput
}  // namespace drake
