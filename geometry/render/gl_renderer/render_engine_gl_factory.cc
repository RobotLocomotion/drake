#include "drake/geometry/render/gl_renderer/render_engine_gl_factory.h"

#include "drake/geometry/render/gl_renderer/render_engine_gl.h"

namespace drake {
namespace geometry {
namespace render {

std::unique_ptr<RenderEngine> MakeRenderEngineGl() {
  return std::make_unique<RenderEngineGl>();
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
