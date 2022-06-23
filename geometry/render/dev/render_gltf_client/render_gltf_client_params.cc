#include "drake/geometry/render/dev/render_gltf_client/render_gltf_client_params.h"

#include <string>

namespace drake {
namespace geometry {

void RenderEngineGltfClientParams::Validate() const {
  if (base_url.empty()) {
    throw std::logic_error(
        "RenderEngineGltfClientParams: base_url may not be empty.");
  }

  auto all_slashes = [](const std::string& str) {
    return std::all_of(str.begin(), str.end(), [](char c){ return c == '/'; });
  };

  if (all_slashes(base_url) || all_slashes(render_endpoint)) {
    throw std::logic_error(
        "RenderEngineGltfClientParams: invalid base_url or render_endpoint is "
        "provided that contains only `/`.");
  }
}

std::string RenderEngineGltfClientParams::GetUrl() const {
  std::string url = base_url;
  std::string endpoint = render_endpoint;
  while(url.size() > 0 && url.back() == '/') url.pop_back();
  while(endpoint.size() > 0 && endpoint.front() == '/') endpoint.erase(0, 1);

  return url + "/" + endpoint;
}

}  // namespace geometry
}  // namespace drake
