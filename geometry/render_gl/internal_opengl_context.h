#pragma once

#include <memory>

#include "drake/geometry/render_gl/internal_opengl_includes.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {

/* Handle OpenGL context initialization, clean-up, and generic OpenGL queries.
 This class creates and owns a new context upon construction. Rendering classes
 need to keep their own OpenGlContext and ensure that they switch to it using
 `MakeCurrent()` before any OpenGL calls.
 */
class OpenGlContext {
 public:
  /* Constructor. Initializes a brand new OpenGL context without any objects.

   @param debug  If debug is true, the OpenGl context will be a "debug" context,
   in that the OpenGl implementation's errors will be written to the Drake log.
   See https://www.khronos.org/opengl/wiki/Debug_Output for more information.
   */
  explicit OpenGlContext(bool debug = false);

  /* Copy constructs a context that shares OpenGl objects with `other`.

   - All %OpenGlContext instances share a common display.
   - Each instance has a unique window/OpenGL objects for connecting an OpenGl
     context to an X-windows display.
   - Copying an %OpenGlContext is distinct from creating a new instance in that
     all of the OpenGl objects (buffers, textures, display lists, etc.)
     available to `other` will also be available to the copy. New instances have
     no OpenGl objects. */
  OpenGlContext(const OpenGlContext& other);

  /* All other copy and move semantics are simply deleted. */
  OpenGlContext& operator=(const OpenGlContext& other) = delete;
  OpenGlContext(OpenGlContext&&) = delete;
  OpenGlContext& operator=(OpenGlContext&&) = delete;

  ~OpenGlContext();

  /* Makes this context current.
   @throws std::exception if not successful.  */
  void MakeCurrent() const;

  /* Clears the "current" context from the current thread. Note: if this context
   has been bound in one thread and this called in another thread, the context
   will still be bound in the first thread. */
  static void ClearCurrent();

  /* Reports if this context is bound to the current thread. */
  bool IsCurrent() const;

  /* Displays the window at the given dimensions. Calling this redundantly (on
   an already visible window of the given size) has no effect.  */
  void DisplayWindow(const int width, const int height);

  /* Hides the window (if visible). Calling this on a hidden window has no
   effect.  */
  void HideWindow();

  /* Reports `true` if the window is viewable.

   Being "viewable" is not necessarily the same as visible to the user. The
   window might be occluded. But, at the very least, there is a window to view.

   @throws std::exception if the visibility status cannot be determined. */
  bool IsWindowViewable() const;

  /* Updates the window contents by drawing the back GL buffer into the window.
   */
  void UpdateWindow();

  /* Returns the indicated values for the current OpenGL context.
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
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
