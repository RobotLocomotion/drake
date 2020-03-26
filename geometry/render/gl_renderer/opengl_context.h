#pragma once

#include <memory>

#include "drake/geometry/render/gl_renderer/opengl_includes.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

/** Handle OpenGL context initialization, clean-up, and generic OpenGL queries.
 This class creates and owns a new context upon construction. Rendering classes
 need to keep their own OpenGlContext and ensure that they switch to it using
 `MakeCurrent()` before any OpenGL calls.
 */
class OpenGlContext {
 public:
  /** Constructor. Initializes an OpenGL context.
   @param debug  If debug is true, the OpenGl context will be a "debug" context,
   in that the OpenGl implementation's errors will be written to the Drake log.
   See https://www.khronos.org/opengl/wiki/Debug_Output for more information.
   */
  explicit OpenGlContext(bool debug = false);

  ~OpenGlContext();

  /** Makes this context current.
   @throw std::runtime_error if not successful.  */
  void MakeCurrent();

  /** Returns the specified values for the current OpenGL context.
   @note Even if invoked via an instance, they may not reflect that instance's
   configuration if the instance differs from whichever context is current.  */
  static GLint max_texture_size();
  static GLint max_renderbuffer_size();
  static GLint max_allowable_texture_size();

 private:
  // Note: we are dependent on `GL/glx.h` but don't want to let that bleed into
  // other code. So, we pimpl this up so that glx.h lives only in the
  // implementation.
  class Impl;

  std::unique_ptr<Impl> impl_;
};

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
