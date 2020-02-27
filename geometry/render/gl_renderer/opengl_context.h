#pragma once

#include <memory>

#include "perception/gl_renderer/opengl_includes.h"

namespace anzu {
namespace gl_renderer {

/// Handle OpenGL context initialization, clean-up and generic OpenGL queries.
/// This class can either create and own a new context upon construction, or
/// get a non-owned reference to a context using `get_current()`.
/// Rendering classes need to keep their own OpenGlContext and ensure that they
/// switch to it using `make_current()` before any OpenGL calls.
class OpenGlContext {
 public:
  /// Constructor. Open an X display and initialize an OpenGL's context. The
  /// display will be open and ready for offscreen rendering, but no window is
  /// visible.
  /// @param debug  If true, diagnostics about the initialization of the context
  /// will be written to cerr.
  explicit OpenGlContext(bool debug = false);

  /// Releases context if it is not owned.
  ~OpenGlContext();

  // Gets a "non-owned" context pointer
  static std::unique_ptr<OpenGlContext> get_current();

  /// Makes this context current or throws.
  void make_current() const;

  bool is_initialized() const;

  static GLint max_texture_size();
  static GLint max_renderbuffer_size();
  static GLint max_allowable_texture_size();

 private:
  // Note: we are dependent on `GL/glx.h` but don't want to let that bleed into
  // other code. So, we pimpl this up so that glx.h lives only in the
  // implementation.
  class Impl;

  // For non-owned context.
  explicit OpenGlContext(std::unique_ptr<Impl> impl);

  std::unique_ptr<Impl> impl_;
};

}  // namespace gl_renderer
}  // namespace anzu
