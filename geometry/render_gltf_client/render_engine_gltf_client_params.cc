#include "drake/geometry/render_gltf_client/render_engine_gltf_client_params.h"

#include <string>

namespace drake {
namespace geometry {

std::string RenderEngineGltfClientParams::GetUrl() const {
  std::string url = base_url;
  std::string endpoint = render_endpoint;

  auto all_slashes = [](const std::string& str) {
    if (str.empty()) return false;
    return std::all_of(str.begin(), str.end(), [](char c){ return c == '/'; });
  };

  if (all_slashes(url) || all_slashes(endpoint)) {
    throw std::logic_error(
        "RenderEngineGltfClientParams: invalid base_url or render_endpoint is "
        "provided that contains only `/`.");
  }

  while (url.size() > 0 && url.back() == '/') url.pop_back();
  while (endpoint.size() > 0 && endpoint.front() == '/') endpoint.erase(0, 1);

  return url + "/" + endpoint;
}

}  // namespace geometry
}  // namespace drake
