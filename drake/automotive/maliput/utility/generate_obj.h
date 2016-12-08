#pragma once

#include <string>

#include "drake/automotive/maliput/api/road_geometry.h"

namespace drake {
namespace maliput {
namespace utility {

void GenerateObjFile(const api::RoadGeometry* rg,
                     const std::string& dirpath,
                     const std::string& fileroot,
                     double grid_unit);

}  // namespace utility
}  // namespace maliput
}  // namespace drake
