#include "drake/geometry/render/dev/render_engine_gltf_client_factory.h"

#include "drake/geometry/render/dev/render_engine_gltf_client.h"

namespace drake {
namespace geometry {
namespace render {

std::unique_ptr<RenderEngine> MakeRenderEngineGltfClient(
    const RenderEngineGltfClientParams& params) {
  return std::make_unique<internal::RenderEngineGltfClient>(params);
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
