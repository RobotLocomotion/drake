#include "drake/geometry/render_gl/factory.h"

#include <utility>

#include "drake/geometry/render_gl/internal_render_engine_gl.h"

namespace drake {
namespace geometry {

// Definition of extern bool in factory.h. When we build against *this* .cc file
// RenderEngineGl is always available.
const bool kHasRenderEngineGl = true;

std::unique_ptr<render::RenderEngine> MakeRenderEngineGl(
    RenderEngineGlParams params) {
  return std::make_unique<render_gl::internal::RenderEngineGl>(
      std::move(params));
}

}  // namespace geometry
}  // namespace drake
