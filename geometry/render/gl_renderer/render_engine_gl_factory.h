#pragma once

#include <memory>
#include <optional>

#include "drake/geometry/render/render_engine.h"

namespace drake {
namespace geometry {
namespace render {

/** Constructs an optimized simple renderer based on direct calls to the OpenGL
 API. All geometries with perception role are added to this renderer.  */
std::unique_ptr<RenderEngine> MakeRenderEngineGl();

}  // namespace render
}  // namespace geometry
}  // namespace drake
