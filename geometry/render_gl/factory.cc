#include "drake/geometry/render_gl/factory.h"

#include <utility>

#include "drake/geometry/render_gl/internal_render_engine_gl.h"

namespace drake {
namespace geometry {
namespace render {

std::unique_ptr<RenderEngine> MakeRenderEngineGl(RenderEngineGlParams params) {
  return std::make_unique<internal::RenderEngineGl>(std::move(params));
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
