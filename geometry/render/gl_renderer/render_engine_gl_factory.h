#pragma once

#include <memory>
#include <optional>

#include "drake/geometry/render/render_engine.h"

namespace drake {
namespace geometry {
namespace render {

/**
Constructs a RenderEngine implementation which uses a purely OpenGL
renderer. The engine only works under Ubuntu. If called on a Mac, it will
produce a "dummy" implementation. */
std::unique_ptr<RenderEngine> MakeRenderEngineGl();

}  // namespace render
}  // namespace geometry
}  // namespace drake
