#include "drake/geometry/render/render_camera_properties.h"

#include <utility>

#include <fmt/format.h>

namespace drake {
namespace geometry {
namespace render {

RenderCameraProperties::RenderCameraProperties(std::string render_engine_name)
    : RenderCameraProperties(std::move(render_engine_name), 0.01, 10.0) {}

RenderCameraProperties::RenderCameraProperties(std::string render_engine_name,
                                               double near_clipping,
                                               double far_clipping)
    : render_engine_name_(std::move(render_engine_name)),
      near_clipping_(near_clipping),
      far_clipping_(far_clipping) {
  if (near_clipping <= 0 || far_clipping <= 0 ||
      far_clipping <= near_clipping) {
    throw std::runtime_error(
        fmt::format("The clipping plane values must both be positive and "
                    "the far clipping plane must be greater than the "
                    "near. Instantiated with near = {} and far = {}",
                    near_clipping, far_clipping));
  }
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
