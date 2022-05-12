#include "drake/geometry/render_vtk/render_engine_vtk_factory.h"

#include "drake/geometry/render_vtk/internal_render_engine_vtk.h"

namespace drake {
namespace geometry {
namespace render_vtk {

std::unique_ptr<render::RenderEngine> MakeRenderEngineVtk(
    const RenderEngineVtkParams& params) {
  return std::make_unique<render::RenderEngineVtk>(params);
}

}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
