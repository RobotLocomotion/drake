#include "drake/geometry/render/gl_renderer/render_engine_gl_factory.h"

#include <utility>

#include "drake/geometry/render/gl_renderer/render_engine_gl.h"

namespace drake {
namespace geometry {
namespace render {

std::unique_ptr<RenderEngine> MakeRenderEngineGl(RenderEngineGlParams params) {
  return std::make_unique<RenderEngineGl>(std::move(params));
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
