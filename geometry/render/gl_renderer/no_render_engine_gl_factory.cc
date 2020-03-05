#include "drake/geometry/render/gl_renderer/render_engine_gl_factory.h"

namespace drake {
namespace geometry {
namespace render {

std::unique_ptr<RenderEngine> MakeRenderEngineGl() {
  throw std::runtime_error(
      "RenderEngineGl was not compiled. You'll need to use a different render "
      "engine.");
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
