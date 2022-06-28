#include "drake/geometry/render_gltf_client/render_engine_gltf_client_params.h"

#include <string>

namespace drake {
namespace geometry {

std::string RenderEngineGltfClientParams::GetUrl() const {
  std::string url = base_url;
  std::string endpoint = render_endpoint;
  while (url.size() > 0 && url.back() == '/') {
    url.pop_back();
  }
  while (endpoint.size() > 0 && endpoint.front() == '/') {
    endpoint.erase(0, 1);
  }
  return url + "/" + endpoint;
}

}  // namespace geometry
}  // namespace drake
