#include "drake/geometry/render/render_camera_properties.h"

#include <fmt/format.h>

namespace drake {
namespace geometry {
namespace render {

DepthRange::DepthRange(double min_in, double max_in)
    : min_depth_(min_in), max_depth_(max_in) {
  if (min_depth_ <= 0 || max_depth_ <= 0 || max_depth_ <= min_depth_) {
    throw std::runtime_error(
        fmt::format("The depth range values must both be positive and "
                    "the maximum depth must be greater than the minimum depth. "
                    "Instantiated with min = {} and max = {}",
                    min_depth_, max_depth_));
  }
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
