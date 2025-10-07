#include "drake/geometry/render_vtk/factory.h"

#include "drake/geometry/render_vtk/internal_render_engine_vtk.h"

namespace drake {
namespace geometry {

// Definition of extern bool in factory.h. When we build against *this* .cc file
// RenderEngineVtk is always available.
const bool kHasRenderEngineVtk = true;

std::unique_ptr<render::RenderEngine> MakeRenderEngineVtk(
    const RenderEngineVtkParams& params) {
  return std::make_unique<render_vtk::internal::RenderEngineVtk>(params);
}

}  // namespace geometry
}  // namespace drake
