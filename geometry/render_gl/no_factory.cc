#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_gl/render_engine_gl_params.h"

namespace drake {
namespace geometry {

// Definition of extern bool in factory.h. When we build against *this* .cc file
// RenderEngineGl is not available.
const bool kHasRenderEngineGl = false;

std::unique_ptr<render::RenderEngine> MakeRenderEngineGl(RenderEngineGlParams) {
  throw std::runtime_error(
      "RenderEngineGl was not compiled. You'll need to use a different render "
      "engine.");
}

namespace render {
  DRAKE_DEPRECATED("2023-08-01", "Use the geometry namespace instead.")
  const bool kHasRenderEngineGl = false;
}  // namespace render

}  // namespace geometry
}  // namespace drake
