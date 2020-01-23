#pragma once

#include <memory>
#include <optional>

#include "drake/geometry/render/render_engine.h"

namespace drake {
namespace geometry {
namespace render {

/** Constructs a RenderEngine implementation which uses a purely OpenGL
 renderer. Currently this uses a temporary dummy implementation. Due to the lack
 of support on Mac, this has been initially introduced to verify platform
 specific (i.e. Ubuntu only) conditionals for building and installing. The rest
 of the implementation will be ported over once the conditionalizing is no
 longer considered a risk.
 <!-- TODO(tehbelinda): Complete port of RenderEngineGl. -->
 */
std::unique_ptr<RenderEngine> MakeRenderEngineGl();

}  // namespace render
}  // namespace geometry
}  // namespace drake
