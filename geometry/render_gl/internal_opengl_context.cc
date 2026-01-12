#include "drake/geometry/render_gl/internal_opengl_context.h"

#include <algorithm>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

// Note: These are intentionally included here since they are only needed at the
// implementation level, and not in a grouping of more generic headers like
// opengl_includes.h. See opengl_context.h for where pimpl is applied.
#include <vtkX11Functions.h>
#include <vtkglad/include/glad/glx.h>

#include "drake/common/drake_assert.h"
#include "drake/common/scope_exit.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/geometry/render_gl/internal_loaders.h"

namespace drake {
namespace geometry {
namespace render_gl {
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
GLXContext glXCreateContextAttribsARB(Display* dpy, GLXFBConfig config,
                                      GLXContext share_context, Bool direct,
                                      const int* attrib_list) {
  return GetGlXFunctionArb<GLXContext(Display*, GLXFBConfig, GLXContext, Bool,
                                      const int*)>(
      "glXCreateContextAttribsARB")(dpy, config, share_context, direct,
                                    attrib_list);
}

// Returns the GL_VENDOR bytes as a std::string.
std::string GetGlVendor() {
  std::stringstream result;
  for (const GLubyte* iter = glGetString(GL_VENDOR); *iter != 0; ++iter) {
    result << static_cast<char>(*iter);
  }
  return result.str();
}

void GlDebugCallback(GLenum, GLenum type, GLuint, GLenum severity, GLsizei,
                     const GLchar* message, const void*) {
  constexpr const char* output =
      "GL CALLBACK: {:s} type = 0x{:x}, severity = 0x{:x}, message = {:s}";
  if (type == GL_DEBUG_TYPE_ERROR) {
    drake::log()->error(output, "** GL_ERROR **", type, severity, message);
  } else {
    drake::log()->info(output, "", type, severity, message);
  }
}

static Display* display() {
  static Display* g_display = static_cast<Display*>(GladLoaderLoadGlx());
  return g_display;
}

}  // namespace

class OpenGlContext::Impl {
 public:
  // Open an X display and initialize an OpenGL context. The display will be
  // open and ready for offscreen rendering, but no window is visible.
  explicit Impl(bool debug, GLXContext source_context = NULL) : debug_(debug) {
    // We must do this first to lazy-init X11 and GLX.
    display();

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
    const int screen_id = vtkXDefaultScreen(display());

    // No matter what, we want to make sure that the context is not current
    // at the conclusion of construction.
    ScopeExit unbind([]() {
      OpenGlContext::ClearCurrent();
    });

    GLXFBConfig* fb_configs =
        glXChooseFBConfig(display(), screen_id, kVisualAttribs, &fb_count);
    ScopeExit guard([fb_configs]() {
      vtkXFree(fb_configs);
    });
    if (fb_configs == nullptr) {
      throw std::runtime_error(
          "Error initializing OpenGL Context for RenderEngineGL; no suitable "
          "frame buffer configuration found.");
    }
    // Since we have provided attributes in the call to glXChooseFBConfig, we
    // are guaranteed to have a valid result in fb_configs as we have already
    // checked for null.
    DRAKE_DEMAND(fb_count > 0);

    // Set up window for displaying render results.
    XVisualInfo* visual = glXGetVisualFromFBConfig(display(), fb_configs[0]);
    if (visual == nullptr) {
      throw std::runtime_error(
          "Unable to generate an OpenGl display window; visual info "
          "unavailable.");
    }
    ScopeExit visual_guard([visual]() {
      vtkXFree(visual);
    });
    XSetWindowAttributes window_attribs;

    // Track to see if we have successful initialization; if not, we use this
    // to free up resources who don't have their own destructors in the case
    // that the constructor exits badly (e.g., via exception).
    bool is_complete = false;

    // This requires a call to XFreeColormap in the destructor.
    window_attribs.colormap =
        vtkXCreateColormap(display(), vtkXRootWindow(display(), screen_id),
                           visual->visual, AllocNone);
    ScopeExit colormap_guard(
        [colormap = window_attribs.colormap, &is_complete]() {
          if (!is_complete) vtkXFreeColormap(display(), colormap);
        });

    // Enable just the Expose event so we know when the window is ready to be
    // redrawn.
    window_attribs.event_mask = ExposureMask;
    // This requires a call to XDestroyWindow in the destructor.
    window_ = vtkXCreateWindow(display(), vtkXRootWindow(display(), screen_id),
                               0, 0, window_width_, window_height_, 0,
                               visual->depth, InputOutput, visual->visual,
                               CWColormap | CWEventMask, &window_attribs);
    ScopeExit window_guard([window = window_, &is_complete]() {
      if (!is_complete) vtkXDestroyWindow(display(), window);
    });

    // Create an OpenGL context.
    const int kContextAttribs[] = {GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
                                   GLX_CONTEXT_MINOR_VERSION_ARB, 3, None};
    // NOTE: The consts True and False come from gl/glx.h (indirectly), but
    // ultimately from X11/Xlib.h.
    // This requires a call to glXDestroyContext in the destructor.
    context_ = glXCreateContextAttribsARB(
        display(), fb_configs[0], source_context, True, kContextAttribs);
    if (context_ == nullptr) {
      throw std::runtime_error(
          "Error initializing OpenGL Context for RenderEngineGL; failed to "
          "create context via glXCreateContextAttribsARB.");
    }
    ScopeExit context_guard([context = context_, &is_complete]() {
      if (!is_complete) glXDestroyContext(display(), context);
    });

    vtkXSync(display(), False);

    // Enable debug.
    if (debug) {
      MakeCurrent();
      drake::log()->info("Vendor: {}", GetGlVendor());
      glEnable(GL_DEBUG_OUTPUT);
      glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
      glDebugMessageCallback(GlDebugCallback, 0);
    }
    is_complete = true;
  }

  // Constructs a copy which shares the OpenGl objects stored on the GPU with
  // `other`.
  Impl(const Impl& other) : Impl(other.debug_, other.context_) {}

  ~Impl() {
    glXDestroyContext(display(), context_);
    XWindowAttributes window_attribs;
    vtkXGetWindowAttributes(display(), window_, &window_attribs);
    vtkXFreeColormap(display(), window_attribs.colormap);
    vtkXDestroyWindow(display(), window_);
  }

  void MakeCurrent() const {
    if (glXGetCurrentContext() != context_ &&
        !glXMakeCurrent(display(), window_, context_)) {
      throw std::runtime_error("Error making an OpenGL context current");
    }
    // After the first time we call glXMakeCurrent, we need to load OpenGL.
    static const int kGlVersion = []() {
      const int version = gladLoaderLoadGL();
      if (version == 0) {
        throw std::runtime_error("Could not dlopen libGL.so.1");
      }
      return version;
    }();
    unused(kGlVersion);
  }

  bool IsCurrent() const { return glXGetCurrentContext() == context_; }

  void DisplayWindow(const int width, const int height) {
    if (width != window_width_ || height != window_height_) {
      vtkXResizeWindow(display(), window_, width, height);
      // If the window isn't viewable, it won't send an expose event.
      if (IsWindowViewable()) WaitForExposeEvent();
      window_width_ = width;
      window_height_ = height;
    }
    if (!IsWindowViewable()) {
      vtkXMapRaised(display(), window_);
      WaitForExposeEvent();
    }
    // We wait for confirmation events to make sure we don't attempt to draw
    // to the window that isn't available yet. Without waiting for the events,
    // the XServer could process the events in such a way that the rendering
    // doesn't make it to the visible window (based on race conditions in the
    // XServer).
  }

  void HideWindow() {
    if (IsWindowViewable()) {
      vtkXUnmapWindow(display(), window_);
      // Unmapping a window provides no events on that window.
    }
  }

  bool IsWindowViewable() const {
    XWindowAttributes attr;
    const Status status = vtkXGetWindowAttributes(display(), window_, &attr);

    // In xlib, a zero status implies function failure.
    // https://tronche.com/gui/x/xlib/introduction/errors.html#Status
    if (status == 0) {
      throw std::runtime_error(
          "Unable to determine the status of the window associated with the "
          "OpenGl context");
    }
    return attr.map_state == IsViewable;
  }

  void UpdateWindow() {
    vtkXClearWindow(display(), window_);
    glXSwapBuffers(display(), window_);
  }

  // Waits for the display() to transmit an Expose event.
  void WaitForExposeEvent() const {
    XEvent event;
    // This blocks until the window gets an "Expose" event.
    vtkXWindowEvent(display(), window_, ExposureMask, &event);
    DRAKE_DEMAND(event.type == Expose);
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
  GLXContext context_{nullptr};

  // The associated window to support display of rendering results.
  Window window_;
  // The window's current size. We default it to an arbitrary 10x10 (a size a
  // user is unlikely to call). XCreateWindow (called in the constructor) has
  // undocumented behavior for zero-area windows, causing the constructor to
  // fail or hang. Empirically, 10x10 allows things to work. The reason is
  // not yet understood.
  int window_width_{10};
  int window_height_{10};

  const bool debug_{};
};

OpenGlContext::OpenGlContext(bool debug)
    : impl_(new OpenGlContext::Impl(debug)) {}

OpenGlContext::OpenGlContext(const OpenGlContext& other)
    : impl_(std::make_unique<OpenGlContext::Impl>(*other.impl_)) {}

OpenGlContext::~OpenGlContext() = default;

void OpenGlContext::MakeCurrent() const {
  impl_->MakeCurrent();
}

void OpenGlContext::ClearCurrent() {
  glXMakeCurrent(display(), None, NULL);
}

bool OpenGlContext::IsCurrent() const {
  return impl_->IsCurrent();
}

void OpenGlContext::DisplayWindow(const int width, const int height) {
  impl_->DisplayWindow(width, height);
}

void OpenGlContext::HideWindow() {
  impl_->HideWindow();
}

bool OpenGlContext::IsWindowViewable() const {
  return impl_->IsWindowViewable();
}

void OpenGlContext::UpdateWindow() {
  impl_->UpdateWindow();
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
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
