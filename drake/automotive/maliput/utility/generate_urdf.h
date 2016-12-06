#pragma once

#include "drake/automotive/maliput/api/road_geometry.h"

namespace drake {
namespace maliput {
namespace utility {

void generate_urdf(const std::string& dirname,
                   const std::string& fileroot,
                   const api::RoadGeometry* rg,
                   const double grid_unit);

}  // namespace utility
}  // namespace maliput
}  // namespace drake
