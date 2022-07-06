#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_gl/render_engine_gl_params.h"

namespace drake {
namespace geometry {
namespace render {

std::unique_ptr<RenderEngine> MakeRenderEngineGl(RenderEngineGlParams) {
  throw std::runtime_error(
      "RenderEngineGl was not compiled. You'll need to use a different render "
      "engine.");
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
