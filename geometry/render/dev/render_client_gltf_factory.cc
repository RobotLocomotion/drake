#include "drake/geometry/render/dev/render_client_gltf_factory.h"

#include "drake/geometry/render/dev/render_client_gltf.h"

namespace drake {
namespace geometry {
namespace render {

std::unique_ptr<RenderEngine> MakeRenderClientGLTF(
    const RenderClientGLTFParams& params) {
  return std::make_unique<RenderClientGLTF>(params);
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
