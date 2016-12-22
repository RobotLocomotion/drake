#pragma once

#include <string>

#include "drake/automotive/maliput/api/road_geometry.h"

namespace drake {
namespace maliput {
namespace utility {

/// Generates a Wavefront OBJ model of the road surface of an api::RoadGeometry.
///
/// @param road_geometry  the api::RoadGeometry to model
/// @param dirpath  directory component of the output pathnames
/// @param fileroot  root of the filename component of the output pathnames
/// @param grid_unit  distance between vertices in the output mesh
///
/// GenerateObjFile actually produces two files:  the first, named
/// [@p dirpath]/[@p fileroot].obj, is a Wavefront OBJ containing the
/// mesh which models the api::RoadGeometry.  The second file is a
/// Wavefront MTL file named [@p dirpath]/[@p fileroot].mtl, containing
/// descriptions of materials referenced by the OBJ file.
///
/// The produced mesh covers the area within the lane-bounds of every
/// api::Lane in the RoadGeometry.
void GenerateObjFile(const api::RoadGeometry* road_geometry,
                     const std::string& dirpath,
                     const std::string& fileroot,
                     double grid_unit);

}  // namespace utility
}  // namespace maliput
}  // namespace drake
