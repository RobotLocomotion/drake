#pragma once

#include <string>

#include "drake/automotive/maliput/api/road_geometry.h"

namespace drake {
namespace maliput {
namespace utility {

/// Generates a URDF file which models the road surface of an api::RoadGeometry.
///
/// @param rg  the api::RoadGeometry to model
/// @param dirpath  directory component of the output pathnames
/// @param fileroot  root of the filename component of the output pathnames
/// @param grid_unit  distance between vertices in the output mesh
///
/// GenerateUrdfFile() actually produces three files:
///  - [@p dirpath]/[@p fileroot].urdf the URDF file, which is little more
///    than a wrapper for an OBJ file;
///  - [@p dirpath]/[@p fileroot].obj a Wavefront OBJ file containing the
///    visual mesh which models the surface;
///  - [@p dirpath]/[@p fileroot].mtl a Wavefront MTL file describing
///    the materials referenced by the OBJ file.
///
/// The produced mesh covers the area within the lane-bounds of every
/// api::Lane in the RoadGeometry.
void GenerateUrdfFile(const std::string& dirname,
                      const std::string& fileroot,
                      const api::RoadGeometry* rg,
                      double grid_unit);

}  // namespace utility
}  // namespace maliput
}  // namespace drake
