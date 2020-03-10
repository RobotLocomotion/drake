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
#include "drake/common/text_logging.h"

namespace drake {
namespace geometry {
namespace render {
namespace gl {

namespace {

// Helper function for loading OpenGL extension functions.
template <class F>
F* GetGLXFunctionARB(const char* func_name) {
  // We must copy the string to a GLubyte buffer to avoid strict aliasing rules.
  // https://gist.github.com/shafik/848ae25ee209f698763cffee272a58f8
  constexpr int kBufferSize = 128;
  DRAKE_ASSERT(strlen(func_name) < kBufferSize);
  GLubyte gl_func_name[kBufferSize] = {};
  std::memcpy(gl_func_name, func_name, strlen(func_name) + 1);
  return reinterpret_cast<F*>(glXGetProcAddressARB(gl_func_name));
}

void gl_debug_callback(GLenum, GLenum type, GLuint, GLenum severity, GLsizei,
                       const GLchar* message, const void*) {
  drake::log()->error(
      "GL CALLBACK: {:s} type = 0x{:x}, severity = 0x{:x}, message = {:s}",
      (type == GL_DEBUG_TYPE_ERROR ? "** GL_ERROR **" : ""), type, severity,
      message);
}

}  // namespace

class OpenGlContext::Impl {
 public:
  explicit Impl(bool debug = false) {
    // See Offscreen Rendering section here:
    // https://sidvind.com/index.php?title=Opengl/windowless

    // Get framebuffer configs.
    const int kVisualAttribs[] = {GLX_X_RENDERABLE,
                                  True,
                                  GLX_X_VISUAL_TYPE,
                                  GLX_TRUE_COLOR,
                                  GLX_RED_SIZE,
                                  8,
                                  GLX_GREEN_SIZE,
                                  8,
                                  GLX_BLUE_SIZE,
                                  8,
                                  GLX_ALPHA_SIZE,
                                  8,
                                  GLX_DEPTH_SIZE,
                                  24,
                                  GLX_DOUBLEBUFFER,
                                  True,
                                  None};
    int fb_count = 0;
    int screen_id = DefaultScreen(display());
    GLXFBConfig* fb_configs = glXChooseFBConfig(
        display(), screen_id, kVisualAttribs, &fb_count);
    if (fb_configs == nullptr) {
      throw std::runtime_error(
          "Error initializing OpenGL Context for RenderEngineGL; no suitable "
          "frame buffer configuration found.");
    }

    XVisualInfo* visual = glXGetVisualFromFBConfig(display(), fb_configs[0]);
    if (visual == nullptr) {
      throw std::runtime_error(
          "Unable to generate an OpenGl display window; visual info "
          "unavailable.");
    }

    // Set up window for displaying render results.
    XSetWindowAttributes window_attribs;
    window_attribs.border_pixel = BlackPixel(display(), screen_id);
    window_attribs.background_pixel = WhitePixel(display(), screen_id);
    window_attribs.override_redirect = True;
    window_attribs.colormap = XCreateColormap(
        display(), RootWindow(display(), screen_id), visual->visual, AllocNone);
    window_attribs.event_mask = ExposureMask;
    window_ =
        XCreateWindow(display(), RootWindow(display(), screen_id), 0, 0, 320,
                      240, 0, visual->depth, InputOutput, visual->visual,
                      CWBackPixel | CWColormap | CWBorderPixel | CWEventMask,
                      &window_attribs);

    // Create an OpenGL context.
    const int kContextAttribs[] = {GLX_CONTEXT_MAJOR_VERSION_ARB, 4,
                                   GLX_CONTEXT_MINOR_VERSION_ARB, 5, None};
    auto glXCreateContextAttribsARB =
        GetGLXFunctionARB<GLXContext(Display*, GLXFBConfig, GLXContext, Bool,
                                     const int*)>("glXCreateContextAttribsARB");

    // NOTE: The consts True and False come from gl/glx.h (indirectly), but
    // ultimately from X11/Xlib.h.
    context_ = glXCreateContextAttribsARB(display(), fb_configs[0], 0, True,
                                          kContextAttribs);
    if (context_ == nullptr) {
      throw std::runtime_error(
          "Error initializing OpenGL Context for RenderEngineGL; failed to "
          "create context via glXCreateContextAttribsARB.");
    }

    XFree(fb_configs);
    XFree(visual);
    XSync(display(), False);

    // Make it the current context.
    make_current();

    // Debug.
    if (debug) {
      drake::log()->info("Vendor: {}\n",
                         reinterpret_cast<const char*>(glGetString(GL_VENDOR)));
      glEnable(GL_DEBUG_OUTPUT);
      glDebugMessageCallback(gl_debug_callback, 0);
    }
  }

  ~Impl() {
    if (context_ != nullptr && is_owned_) {
      glXDestroyContext(display(), context_);
    }
    if (window_) {
      XWindowAttributes window_attribs;
      XGetWindowAttributes(display(), window_, &window_attribs);
      XFreeColormap(display(), window_attribs.colormap);
      XDestroyWindow(display(), window_);
    }
  }

  void make_current() const {
    if (glXGetCurrentContext() != context_ &&
        !glXMakeCurrent(display(), window_, context_)) {
      throw std::runtime_error("Cannot make context current");
    }
  }

  void display_window(const int width, const int height) {
    XMapRaised(display(), window_);
    if (width != window_width_ || height != window_height_) {
      XResizeWindow(display(), window_, width, height);
      XSync(display(), false);
      window_width_ = width;
      window_height_ = height;
    }
    XClearWindow(display(), window_);
    glXSwapBuffers(display(), window_);
  }

  bool is_initialized() const { return context_ != nullptr; }

  static GLint max_texture_size() {
    GLint res;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &res);
    return res;
  }

  static GLint max_renderbuffer_size() {
    GLint res;
    glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE, &res);
    return res;
  }

 private:
  // Non-owned.
  explicit Impl(GLXContext context) : is_owned_(false), context_(context) {}

  static Display* display() {
    // Turn Display into a singleton to make CI happy, since when we close and
    // reopen the display on CI, we can't request a new OpenGL context.
    // This pattern won't call the corresponding `XCloseDisplay()` when the
    // program exiting, but it seems not so evil to skip that.
    // (https://linux.die.net/man/3/xclosedisplay)
    // TODO(duy): If problems crop up in the future, this can/should be
    // investigated.
    static Display* g_display = XOpenDisplay(0);
    DRAKE_THROW_UNLESS(g_display != nullptr);
    return g_display;
  }

  bool is_owned_{true};
  GLXContext context_{nullptr};
  Window window_;
  int window_width_{0};
  int window_height_{0};
};

OpenGlContext::OpenGlContext(bool debug)
    : impl_(new OpenGlContext::Impl(debug)) {}

OpenGlContext::~OpenGlContext() = default;

void OpenGlContext::make_current() const { impl_->make_current(); }

void OpenGlContext::display_window(const int width, const int height) {
  impl_->display_window(width, height);
}

bool OpenGlContext::is_initialized() const { return impl_->is_initialized(); }

GLint OpenGlContext::max_texture_size() {
  return OpenGlContext::Impl::max_texture_size();
}

GLint OpenGlContext::max_renderbuffer_size() {
  return OpenGlContext::Impl::max_renderbuffer_size();
}

GLint OpenGlContext::max_allowable_texture_size() {
  // TODO(duy): Take into account CUDA limits.
  return std::min(OpenGlContext::max_texture_size(),
                  OpenGlContext::max_renderbuffer_size());
}

}  // namespace gl
}  // namespace render
}  // namespace geometry
}  // namespace drake
