#pragma once

#include "drake/automotive/maliput/api/road_geometry.h"

namespace drake {
namespace maliput {
namespace utility {

void generate_obj(const api::RoadGeometry* rg,
                  const std::string& filename,
                  double grid_unit);

}  // namespace utility
}  // namespace maliput
}  // namespace drake
