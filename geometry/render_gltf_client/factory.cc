#include "drake/geometry/render_gltf_client/factory.h"

#include "drake/common/network_policy.h"
#include "drake/geometry/render_gltf_client/internal_render_engine_gltf_client.h"

namespace drake {
namespace geometry {

// Definition of extern bool in factory.h. When we build against *this* .cc file
// RenderEngineGltfClient is available conditionally (but still governed by
// DRAKE_ALLOW_NETWORK at runtime).
const bool kHasRenderEngineGltfClient = true;

std::unique_ptr<render::RenderEngine> MakeRenderEngineGltfClient(
    const RenderEngineGltfClientParams& params) {
  if (!drake::internal::IsNetworkingAllowed("render_gltf_client")) {
    throw std::runtime_error(
        "RenderEngineGltfClient has been disabled via the DRAKE_ALLOW_NETWORK "
        "environment variable");
  }

  return std::make_unique<render_gltf_client::internal::RenderEngineGltfClient>(
      params);
}

}  // namespace geometry
}  // namespace drake
