#include "drake/geometry/render/gl_renderer/dev/render_engine_gl_factory.h"

#include "drake/geometry/render/gl_renderer/dev/render_engine_gl.h"

namespace drake {
namespace geometry {
namespace render {
namespace dev {

std::unique_ptr<RenderEngine> MakeRenderEngineGl() {
  return std::make_unique<internal::RenderEngineGl>();
}

}  // namespace dev
}  // namespace render
}  // namespace geometry
}  // namespace drake
