#include "drake/geometry/render_vtk/factory.h"

#include "drake/geometry/render_vtk/internal_render_engine_vtk.h"

namespace drake {
namespace geometry {

std::unique_ptr<render::RenderEngine> MakeRenderEngineVtk(
    const RenderEngineVtkParams& params) {
  return std::make_unique<render_vtk::internal::RenderEngineVtk>(params);
}

}  // namespace geometry
}  // namespace drake
