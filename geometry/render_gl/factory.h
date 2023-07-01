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

 <b> Using RenderEngineGl in multiple threads </b>

 Most importantly, a single %RenderEngineGl should *not* be exercised in
 multiple threads. One thread, one %RenderEngineGl instance.

 A %RenderEngineGl instance and its *clones* can be used in different threads
 simultaneously, but *only* the rendering APIs are threadsafe. Do not mutate the
 contents of the engine (e.g., adding/removing geometries, etc.) in parallel.

 Two independently constructed %RenderEngineGl instances can be freely used in
 different threads -- all APIs are available.

 The expected workflow is to add a %RenderEngineGl instance a SceneGraph
 instance (see SceneGraph::AddRenderer()) and then to populate SceneGraph with
 the desired geometry. Each systems::Context allocated for that SceneGraph will
 receive a clone of the original %RenderEngineGl. One systems::Context can be
 used per thread to create rendered images in parallel.

 @throws std::exception if kHasRenderEngineGl is false. */
std::unique_ptr<render::RenderEngine> MakeRenderEngineGl(
    RenderEngineGlParams params = {});

}  // namespace geometry
}  // namespace drake
