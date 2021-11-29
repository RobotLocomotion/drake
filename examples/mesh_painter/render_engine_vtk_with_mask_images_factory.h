#pragma once

#include <memory>

#include "drake/geometry/render/render_engine_vtk_factory.h"

namespace drake {
namespace examples {
namespace mesh_painter {

/** See geometry::render::MakeRenderEngineVtk. */
std::unique_ptr<geometry::render::RenderEngine>
MakeRenderEngineVtkWithMaskImages(
    const geometry::render::RenderEngineVtkParams& params);

}  // namespace mesh_painter
}  // namespace examples
}  // namespace drake
