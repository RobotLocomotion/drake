#include <memory>

#include "drake/geometry/render_gltf_client/factory.h"
#include "drake/geometry/render_gltf_client/render_engine_gltf_client_params.h"

namespace drake {
namespace geometry {

// Definition of extern bool in factory.h. When we build against *this* .cc file
// RenderEngineGltfClient is not available.
const bool kHasRenderEngineGltfClient = false;

std::unique_ptr<render::RenderEngine> MakeRenderEngineGltfClient(
    const RenderEngineGltfClientParams&) {
  throw std::runtime_error(
      "RenderEngineGltfClient was not compiled. You'll need to use a different "
      "render engine.");
}

}  // namespace geometry
}  // namespace drake
