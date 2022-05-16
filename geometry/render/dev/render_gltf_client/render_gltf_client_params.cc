#include "drake/geometry/render/dev/render_gltf_client/render_gltf_client_params.h"

#include <string>

namespace drake {
namespace geometry {

void RenderEngineGltfClientParams::Validate() const {
  if (base_url.empty()) {
    throw std::logic_error(
        "RenderEngineGltfClientParams: base_url may not be empty.");
  }

  if (base_url.back() == '/') {
    throw std::logic_error(
        "RenderEngineGltfClientParams: base_url may not end with '/'.");
  }

  if (render_endpoint.size() >= 1) {
    if (render_endpoint.front() == '/' || render_endpoint.back() == '/') {
      throw std::logic_error(
          "RenderEngineGltfClientParams: render_endpoint may not start or end"
          " with a '/'.");
    }
  }
}

}  // namespace geometry
}  // namespace drake
