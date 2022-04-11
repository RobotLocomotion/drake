#include "drake/geometry/render/dev/render_gltf_client/render_engine_gltf_client_factory.h"

#include "drake/geometry/render/dev/render_gltf_client/render_engine_gltf_client.h"

namespace drake {
namespace geometry {

std::unique_ptr<render::RenderEngine> MakeRenderEngineGltfClient(
    const RenderEngineGltfClientParams& params) {
  return std::make_unique<render_gltf_client::internal::RenderEngineGltfClient>(params);
}

}  // namespace geometry
}  // namespace drake
