#pragma once

#include <memory>
#include <optional>

#include "drake/geometry/render/gl_renderer/render_engine_gl_params.h"
#include "drake/geometry/render/render_engine.h"

namespace drake {
namespace geometry {
namespace render {

/** Constructs a RenderEngine implementation which uses a purely OpenGL
 renderer. The engine only works under Ubuntu. If called on a Mac, it will
 produce a "dummy" implementation.

 @note %RenderEngineGl behaves a bit differently from other RenderEngine
 implementations (e.g., RenderEngineVtk) with respect to displayed images.
 First, %RenderEngineGl can only display a *single* image type at a time. So,
 if a shown window has been requested for both label and color images, the
 images will alternate in the same window. Second, the window display draws all
 images *flipped vertically*. The image produced will be compatible with the
 Drake ecosystem, only the visualization will be upside down. This has been
 documented in https://github.com/RobotLocomotion/drake/issues/14254.

 @warning %RenderEngineGl is not threadsafe. If a SceneGraph is instantiated
 with a RenderEngineGl and there are multiple Context instances for that
 SceneGraph, rendering in multiple threads may exhibit issues.  */
std::unique_ptr<RenderEngine> MakeRenderEngineGl(
    RenderEngineGlParams params = {});

}  // namespace render
}  // namespace geometry
}  // namespace drake
