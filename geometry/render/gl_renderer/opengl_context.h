#pragma once

#include <memory>

#include "drake/geometry/render/gl_renderer/opengl_includes.h"

namespace drake {
namespace geometry {
namespace render {
namespace gl {

/** Temporary dummy class for building dummy RenderEngineGl dependencies.  */
class OpenGlContext {
 public:
  static void Dummy() {}
};

}  // namespace gl
}  // namespace render
}  // namespace geometry
}  // namespace drake
