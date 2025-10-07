#include "drake/geometry/render_vtk/render_engine_vtk_params.h"

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {

RenderEngineVtkBackend ParseRenderEngineVtkBackend(
    const RenderEngineVtkParams&) {
    throw std::runtime_error("RenderEngineVtk was not compiled. You'll need to "
        "use a different render engine.");
}

}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
