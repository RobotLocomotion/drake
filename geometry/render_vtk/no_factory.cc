#include <memory>

#include "drake/geometry/render_vtk/factory.h"
#include "drake/geometry/render_vtk/render_engine_vtk_params.h"

namespace drake {
namespace geometry {

// Definition of extern bool in factory.h. When we build against *this* .cc file
// RenderEngineVtk is not available.
const bool kHasRenderEngineVtk = false;

std::unique_ptr<render::RenderEngine> MakeRenderEngineVtk(
    const RenderEngineVtkParams&) {
  throw std::runtime_error(
      "RenderEngineVtk was not compiled. You'll need to use a different render "
      "engine.");
}

}  // namespace geometry
}  // namespace drake
