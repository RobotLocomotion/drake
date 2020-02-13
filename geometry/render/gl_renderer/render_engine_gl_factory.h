#pragma once

#include <memory>
#include <optional>

#include "drake/geometry/render/render_engine.h"

namespace drake {
namespace geometry {
namespace render {

/** Constructs an optimized simple renderer based on direct calls to the OpenGL
 API. For now, it only renders depth images, i.e. no color or label images are
 supported. The presence of PerceptionProperties is enough to register the
 geometry as no specific properties are required.   */
std::unique_ptr<RenderEngine> MakeRenderEngineGl();

}  // namespace render
}  // namespace geometry
}  // namespace drake
