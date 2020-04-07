#include "drake/geometry/render/gl_renderer/opengl_context.h"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

// Note: This is intentionally included here since it's only needed at the
// implementation level, and not in a grouping of more generic headers like
// opengl_includes.h. See opengl_context.h for where pimpl is applied.
#include <GL/glx.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/scope_exit.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

namespace {

// Helper function for loading OpenGL extension functions. For more info, see:
// https://www.opengl.org/archives/resources/features/OGLextensions.
template <class F>
F* GetGlXFunctionArb(const char* func_name) {
  // We must copy the string to a GLubyte buffer to avoid strict aliasing rules.
  // https://gist.github.com/shafik/848ae25ee209f698763cffee272a58f8
  constexpr int kBufferSize = 128;
  DRAKE_DEMAND(strlen(func_name) < kBufferSize);
  GLubyte gl_func_name[kBufferSize] = {};
  std::memcpy(gl_func_name, func_name, strlen(func_name) + 1);
  F* result = reinterpret_cast<F*>(glXGetProcAddressARB(gl_func_name));
  DRAKE_DEMAND(result != nullptr);
  return result;
}

// Wrapper that calls the GLX function of the same name as if it has a proper
// header file declaration (instead of the dynamic lookup that happens under
// the hood) to improve readability and debug-ability.  For API docs see
// https://www.khronos.org/registry/OpenGL/extensions/ARB/GLX_ARB_create_context.txt
GLXContext glXCreateContextAttribsARB(
    Display* dpy, GLXFBConfig config, GLXContext share_context, Bool direct,
    const int* attrib_list) {
  return GetGlXFunctionArb<
      GLXContext(Display*, GLXFBConfig, GLXContext, Bool, const int*)>(
      "glXCreateContextAttribsARB")(
          dpy, config, share_context, direct, attrib_list);
}

void GlDebugCallback(GLenum, GLenum type, GLuint, GLenum severity, GLsizei,
                     const GLchar* message, const void*) {
  const char* output =
      "GL CALLBACK: {:s} type = 0x{:x}, severity = 0x{:x}, message = {:s}";
  if (type == GL_DEBUG_TYPE_ERROR) {
    drake::log()->error(output, "** GL_ERROR **", type, severity, message);
  } else {
    drake::log()->info(output, "", type, severity, message);
  }
}

}  // namespace

class OpenGlContext::Impl {
 public:
  // Open an X display and initialize an OpenGL context. The display will be
  // open and ready for offscreen rendering, but no window is visible.
  explicit Impl(bool debug) {
    // See Offscreen Rendering section here:
    // https://sidvind.com/index.php?title=Opengl/windowless

    // Get framebuffer configs.
    const int kVisualAttribs[] = {None};
    int fb_count = 0;
    GLXFBConfig* fb_configs = glXChooseFBConfig(
        display(), DefaultScreen(display()), kVisualAttribs, &fb_count);
    ScopeExit guard([fb_configs]() { XFree(fb_configs); });
    if (fb_configs == nullptr) {
      throw std::runtime_error(
          "Error initializing OpenGL Context for RenderEngineGL; no suitable "
          "frame buffer configuration found.");
    }

    // Create an OpenGL context.
    const int kContextAttribs[] = {GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
                                   GLX_CONTEXT_MINOR_VERSION_ARB, 3, None};
    // Since we have provided attributes in the call to glXChooseFBConfig, we
    // are guaranteed to have a valid result in fb_configs as we have already
    // checked for null.
    DRAKE_DEMAND(fb_count > 0);
    // NOTE: The consts True and False come from gl/glx.h (indirectly), but
    // ultimately from X11/Xlib.h.
    context_ = glXCreateContextAttribsARB(display(), fb_configs[0], 0, True,
                                          kContextAttribs);
    if (context_ == nullptr) {
      throw std::runtime_error(
          "Error initializing OpenGL Context for RenderEngineGL; failed to "
          "create context via glXCreateContextAttribsARB.");
    }

    XSync(display(), False);

    // Make it the current context.
    MakeCurrent();

    // Enable debug.
    if (debug) {
      drake::log()->info("Vendor: {}", glGetString(GL_VENDOR));
      glEnable(GL_DEBUG_OUTPUT);
      glDebugMessageCallback(GlDebugCallback, 0);
    }
  }

  ~Impl() {
    glXDestroyContext(display(), context_);
  }

  void MakeCurrent() {
    if (glXGetCurrentContext() != context_ &&
        !glXMakeCurrent(display(), None, context_)) {
      throw std::runtime_error("Error making an OpenGL context current");
    }
  }

  static GLint max_texture_size() {
    GLint res{-1};
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &res);
    return res;
  }

  static GLint max_renderbuffer_size() {
    GLint res{-1};
    glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE, &res);
    return res;
  }

  static GLint max_allowable_texture_size() {
    // TODO(duy): Take into account CUDA limits.
    return std::min(max_texture_size(), max_renderbuffer_size());
  }

 private:
  static Display* display() {
    // Turn Display into a singleton to make CI happy, since when we close and
    // reopen the display on CI, we can't request a new OpenGL context.
    // This pattern won't call the corresponding `XCloseDisplay()` when the
    // program exits, but it seems not so evil to skip that.
    // (https://linux.die.net/man/3/xclosedisplay)
    // TODO(duy): If problems crop up in the future, this can/should be
    // investigated.
    static Display* g_display = XOpenDisplay(0);
    DRAKE_THROW_UNLESS(g_display != nullptr);
    return g_display;
  }

  GLXContext context_{nullptr};
};

OpenGlContext::OpenGlContext(bool debug)
    : impl_(new OpenGlContext::Impl(debug)) {}

OpenGlContext::~OpenGlContext() = default;

void OpenGlContext::MakeCurrent() { impl_->MakeCurrent(); }

GLint OpenGlContext::max_texture_size() {
  return OpenGlContext::Impl::max_texture_size();
}

GLint OpenGlContext::max_renderbuffer_size() {
  return OpenGlContext::Impl::max_renderbuffer_size();
}

GLint OpenGlContext::max_allowable_texture_size() {
  return OpenGlContext::Impl::max_allowable_texture_size();
}

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
