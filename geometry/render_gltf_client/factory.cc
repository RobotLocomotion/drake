#include "drake/geometry/render_gltf_client/factory.h"

#include "drake/geometry/render_gltf_client/internal_render_engine_gltf_client.h"

namespace drake {
namespace geometry {

std::unique_ptr<render::RenderEngine> MakeRenderEngineGltfClient(
    const RenderEngineGltfClientParams& params) {
  return std::make_unique<render_gltf_client::internal::RenderEngineGltfClient>(
      params);
}

}  // namespace geometry
}  // namespace drake
