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
    const int screen_id = DefaultScreen(display());
    GLXFBConfig* fb_configs =
        glXChooseFBConfig(display(), screen_id, kVisualAttribs, &fb_count);
    ScopeExit guard([fb_configs]() { XFree(fb_configs); });
    if (fb_configs == nullptr) {
      throw std::runtime_error(
          "Error initializing OpenGL Context for RenderEngineGL; no suitable "
          "frame buffer configuration found.");
    }
    // Since we have provided attributes in the call to glXChooseFBConfig, we
    // are guaranteed to have a valid result in fb_configs as we have already
    // checked for null. There may be more than one that matches the attributes
    // but we just pick the first for now.
    DRAKE_DEMAND(fb_count > 0);

    // Set up window for displaying render results.
    XVisualInfo* visual = glXGetVisualFromFBConfig(display(), fb_configs[0]);
    if (visual == nullptr) {
      throw std::runtime_error(
          "Unable to generate an OpenGl display window; visual info "
          "unavailable.");
    }
    ScopeExit visual_guard([visual]() { XFree(visual); });
    XSetWindowAttributes window_attribs;
    window_attribs.colormap = XCreateColormap(
        display(), RootWindow(display(), screen_id), visual->visual, AllocNone);
    // Enable just the Expose event so we know when the window is ready to be
    // redrawn.
    window_attribs.event_mask = ExposureMask;
    window_ = XCreateWindow(display(), RootWindow(display(), screen_id), 0, 0,
                            window_width_, window_height_, 0, visual->depth,
                            InputOutput, visual->visual,
                            CWColormap | CWEventMask, &window_attribs);

    // Create an OpenGL context.
    const int kContextAttribs[] = {GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
                                   GLX_CONTEXT_MINOR_VERSION_ARB, 3, None};
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
    XWindowAttributes window_attribs;
    XGetWindowAttributes(display(), window_, &window_attribs);
    XFreeColormap(display(), window_attribs.colormap);
    XDestroyWindow(display(), window_);
  }

  void MakeCurrent() {
    if (glXGetCurrentContext() != context_ &&
        !glXMakeCurrent(display(), window_, context_)) {
      throw std::runtime_error("Error making an OpenGL context current");
    }
  }

  void ResizeWindow(const int width, const int height) {
    if (width != window_width_ || height != window_height_) {
      XResizeWindow(display(), window_, width, height);
      XEvent event;
      // Wait for the Expose event so we know the trigger is complete.
      XNextEvent(display(), &event);
      DRAKE_DEMAND(event.type == Expose);
      window_width_ = width;
      window_height_ = height;
    }
  }

  void DisplayWindow() {
    if (!is_mapped_) {
      XMapRaised(display(), window_);
      is_mapped_ = true;
      XEvent event;
      // Wait for the Expose event so we know the trigger is complete.
      XNextEvent(display(), &event);
      DRAKE_DEMAND(event.type == Expose);
    }
    XClearWindow(display(), window_);
    glXSwapBuffers(display(), window_);
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
  Window window_;
  int window_width_{640};
  int window_height_{480};
  bool is_mapped_{false};
};

OpenGlContext::OpenGlContext(bool debug)
    : impl_(new OpenGlContext::Impl(debug)) {}

OpenGlContext::~OpenGlContext() = default;

void OpenGlContext::MakeCurrent() { impl_->MakeCurrent(); }

void OpenGlContext::ResizeWindow(const int width, const int height) {
  impl_->ResizeWindow(width, height);
}

void OpenGlContext::DisplayWindow() {
  impl_->DisplayWindow();
}

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
