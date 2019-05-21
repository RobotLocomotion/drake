#include "drake/geometry/render/render_engine_vtk_factory.h"

#include "drake/geometry/render/render_engine_vtk.h"

namespace drake {
namespace geometry {
namespace render {

std::unique_ptr<RenderEngine> MakeRenderEngineVtk(
    const RenderEngineVtkParams& params) {
  return std::make_unique<RenderEngineVtk>(params);
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
