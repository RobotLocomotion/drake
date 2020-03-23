#include "drake/geometry/render/gl_renderer/opengl_context.h"

#include <algorithm>

// Note: This is intentionally included here since it's only needed at the
// implementation level, and not in a grouping of more generic headers like
// opengl_includes.h. See opengl_context.h for where pimpl is applied.
#include <GL/freeglut.h>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace geometry {
namespace render {
namespace gl {

namespace {

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

    InitializeGlut();

    // Create a window to get an OpenGL context and hide it since we don't want
    // to display anything yet.
    glutInitWindowSize(window_width_, window_height_);
    glutInitWindowPosition(0, 0);
    window_id_ = glutCreateWindow("Window");
    glutHideWindow();
    auto display = []() {
      glFlush();
    };
    glutDisplayFunc(display);
    GLenum err = glewInit();
    if (GLEW_OK != err) {
      throw std::runtime_error(fmt::format(
          "Error initializing OpenGL Context for RenderEngineGL: {}.",
          glewGetErrorString(err)));
    }

    is_initialized_ = true;

    // Debug.
    if (debug) {
      drake::log()->info("Vendor: {}\n",
                         reinterpret_cast<const char*>(glGetString(GL_VENDOR)));
      glEnable(GL_DEBUG_OUTPUT);
      glDebugMessageCallback(gl_debug_callback, 0);
    }
  }

  ~Impl() {
    if (window_id_ != 0) {
      glutDestroyWindow(window_id_);
    }
  }

  void make_current() const {
    if (glutGetWindow() != window_id_) {
        glutSetWindow(window_id_);
    }
  }

  void resize_window(const int width, const int height) {
    if (width != window_width_ || height != window_height_) {
      glutReshapeWindow(width, height);
      window_width_ = width;
      window_height_ = height;
      glutMainLoopEvent();
    }
  }

  void display_window() {
    glutShowWindow();
    glutMainLoopEvent();
  }

  bool is_initialized() const { return is_initialized_; }

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
  // The GLUT library should only be initialized once so we need to keep track
  // of whether glutInit has already been called.
  static void InitializeGlut() {
      if (!is_glut_initialized_) {
        // Dummy argc and argv values are required arguments for glutInit.
        int argc = 1;
        char* argv[1] = {const_cast<char*>("")};
        glutInit(&argc, argv);
        glutInitContextVersion(3, 3);
        glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
        glutInitContextProfile(GLUT_CORE_PROFILE);
        is_glut_initialized_ = true;
      }
  }
  static bool is_glut_initialized_;

  bool is_initialized_{false};
  int window_id_{0};
  int window_width_{640};
  int window_height_{480};
};
bool OpenGlContext::Impl::is_glut_initialized_{false};

OpenGlContext::OpenGlContext(bool debug)
    : impl_(new OpenGlContext::Impl(debug)) {}

OpenGlContext::~OpenGlContext() = default;

void OpenGlContext::make_current() const { impl_->make_current(); }

void OpenGlContext::display_window() {
  impl_->display_window();
}

void OpenGlContext::resize_window(const int width, const int height) {
  impl_->resize_window(width, height);
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
