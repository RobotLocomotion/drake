#include "drake/geometry/render/dev/render_client_gltf_factory.h"

#include "drake/geometry/render/dev/render_client_gltf.h"

namespace drake {
namespace geometry {
namespace render {

std::unique_ptr<RenderEngine> MakeRenderClientGltf(
    const RenderClientGltfParams& params) {
  return std::make_unique<RenderClientGltf>(params);
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
