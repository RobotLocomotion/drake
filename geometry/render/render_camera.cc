#include "drake/geometry/render/render_camera.h"

#include <utility>

#include <fmt/format.h>

namespace drake {
namespace geometry {
namespace render {

ClippingRange::ClippingRange(double near, double far) : near_(near), far_(far) {
  if (near <= 0 || far <= 0 || far <= near) {
    throw std::runtime_error(fmt::format(
        "The clipping range values must both be positive and far must be "
        "greater than near. Instantiated with near = {} and far = {}",
        near, far));
  }
}

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

DepthRenderCamera::DepthRenderCamera(RenderCameraCore core,
                                     DepthRange depth_range)
    : core_(std::move(core)), depth_range_(std::move(depth_range)) {
  if (depth_range_.min_depth() < core_.clipping().near() ||
      depth_range_.max_depth() > core_.clipping().far()) {
    throw std::runtime_error(fmt::format(
        "Depth camera's depth range extends beyond the clipping planes; near = "
        "{}, far = {}, min. depth = {}, max. depth = {}",
        core_.clipping().near(), core_.clipping().far(),
        depth_range_.min_depth(), depth_range_.max_depth()));
  }
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
