#include "drake/examples/mesh_painter/render_engine_vtk_with_mask_images_factory.h"

#include "drake/examples/mesh_painter/render_engine_vtk_with_mask_images.h"

namespace drake {
namespace examples {
namespace mesh_painter {

std::unique_ptr<geometry::render::RenderEngine>
MakeRenderEngineVtkWithMaskImages(
    const geometry::render::RenderEngineVtkParams& params) {
  return std::make_unique<RenderEngineVtkWithMaskImages>(params);
}

}  // namespace mesh_painter
}  // namespace examples
}  // namespace drake
