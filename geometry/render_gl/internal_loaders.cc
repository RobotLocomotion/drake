#include "drake/geometry/render_gl/internal_loaders.h"

#include <cstdlib>
#include <stdexcept>

#if !defined(__APPLE__)
#include <vtkglad/include/glad/egl.h>
#include <vtkglad/include/glad/glx.h>
#endif

#include <fmt/format.h>

#include "drake/common/drake_throw.h"
#include "drake/common/unused.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {

constexpr char kTroubleshootingUrl[] =
    "https://drake.mit.edu/troubleshooting.html#gl-init";

void GladLoaderLoadEgl() {
#if defined(__APPLE__)
  // This is not reachable via Drake's public API.
  throw std::logic_error("Drake's GladLoaderLoadEgl() is unavailable on macOS");
  unused(kTroubleshootingUrl);
#else
  // Open the library at most once per process. At the time of this
  // writing this does not appear to matter in practice, but we might as
  // well avoid any problems down the road.
  static const int ignored = []() {
    const int version = gladLoaderLoadEGL(EGL_NO_DISPLAY);
    if (version <= 0) {
      throw std::logic_error(
          fmt::format("Drake could not initialize EGL by loading libEGL.so.1; "
                      "please see {} for help.",
                      kTroubleshootingUrl));
    }
    return version;
  }();
  unused(ignored);
#endif
}

void* GladLoaderLoadGlx() {
#if defined(__APPLE__)
  // This is not reachable via Drake's public API.
  throw std::logic_error("Drake's GladLoaderLoadGlx() is unavailable on macOS");
#else
  // Turn Display into a singleton to make CI happy, since when we close and
  // reopen the display on CI, we can't request a new OpenGL context. This
  // pattern won't call the corresponding `XCloseDisplay()` when the program
  // exits (https://linux.die.net/man/3/xclosedisplay), but it seems not so
  // evil to skip that.
  static Display* g_display = []() {
    XInitThreads();
    Display* display = XOpenDisplay(0);
    if (display == nullptr) {
      const char* display_env = std::getenv("DISPLAY");
      throw std::logic_error(
          fmt::format("Drake's could not open the X display DISPLAY={}; "
                      "please see {} for help.",
                      display_env ? std::string{display_env} : std::string{},
                      kTroubleshootingUrl));
    }
    const int version = gladLoaderLoadGLX(display, 0);
    if (version <= 0) {
      throw std::logic_error(
          fmt::format("Drake's could not initialize GLX by loading libGL.so.1; "
                      "please see {} for help.",
                      kTroubleshootingUrl));
    }
    return display;
  }();
  return g_display;
#endif
}

}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
