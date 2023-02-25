#pragma once

#include <memory>

#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render_gl/render_engine_gl_params.h"

namespace drake {
namespace geometry {

/** Reports the availability of the RenderEngineGl implementation. */
extern const bool kHasRenderEngineGl;

/** Constructs a RenderEngine implementation which uses a purely OpenGL
 renderer. The engine only works under Ubuntu. If called on a Mac, it will
 throw.

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
 SceneGraph, rendering in multiple threads may exhibit issues.

 @throws std::exception if kHasRenderEngineGl is false. */
std::unique_ptr<render::RenderEngine> MakeRenderEngineGl(
    RenderEngineGlParams params = {});

}  // namespace geometry
}  // namespace drake
