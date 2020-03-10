#pragma once

#include <memory>

#include "drake/geometry/render/gl_renderer/opengl_includes.h"

namespace drake {
namespace geometry {
namespace render {
namespace gl {

/** Handle OpenGL context initialization, clean-up and generic OpenGL queries.
 This class creates and owns a new context upon construction. Rendering classes
 need to keep their own OpenGlContext and ensure that they switch to it using
 `make_current()` before any OpenGL calls.
 */
class OpenGlContext {
 public:
  /** Constructor. Open an X display and initialize an OpenGL context. The
   display will be open and ready for offscreen rendering, but no window is
   visible.
   @param debug  If true, diagnostics about the initialization of the context
   will be written to the Drake log.
   */
  explicit OpenGlContext(bool debug = false);

  /** Releases context if it is not owned.  */
  ~OpenGlContext();

  /** Makes this context current or throws.  */
  void make_current() const;

  /** Display the render result in a window with the given dimensions.  */
  void display_window(const int width, const int height);

  bool is_initialized() const;

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

}  // namespace gl
}  // namespace render
}  // namespace geometry
}  // namespace drake
