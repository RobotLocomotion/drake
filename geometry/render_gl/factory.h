#pragma once

#include <memory>

#include "drake/common/drake_deprecated.h"
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

 @note %RenderEngineGl has been designed to be threadsafe for *rendering*.
 It requires a unique RenderEngineGl instance per *thread*. In the normal
 workflow, where a %RenderEngineGl has been added to a SceneGraph instance,
 then simply providing a unique systems::Context for each thread will be
 sufficient. Mutating the *contents* of the %RenderEngineGl (e.g., adding or
 removing geometries) is *not* threadsafe.

 @throws std::exception if kHasRenderEngineGl is false. */
std::unique_ptr<render::RenderEngine> MakeRenderEngineGl(
    RenderEngineGlParams params = {});

namespace render {

DRAKE_DEPRECATED("2023-07-01", "Use the geometry namespace instead.")
extern const bool kHasRenderEngineGl;

DRAKE_DEPRECATED("2023-07-01", "Use the geometry namespace instead.")
constexpr auto MakeRenderEngineGl = &geometry::MakeRenderEngineGl;

}  // namespace render

}  // namespace geometry
}  // namespace drake
