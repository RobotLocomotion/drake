#include "drake/geometry/render/gl_renderer/dev/render_engine_gl_factory.h"

namespace drake {
namespace geometry {
namespace render {
namespace dev {

std::unique_ptr<RenderEngine> MakeRenderEngineGl() {
  throw std::runtime_error(
      "RenderEngineGl was not compiled. You'll need to use a different render "
      "engine.");
}

}  // namespace dev
}  // namespace render
}  // namespace geometry
}  // namespace drake
