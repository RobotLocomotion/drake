#include "drake/geometry/render_vtk/render_engine_vtk_params.h"

#include "drake/common/text_logging.h"

namespace drake {
namespace geometry {
namespace internal {

#if defined(__APPLE__)
constexpr bool kApple = true;
#else
constexpr bool kApple = false;
#endif

// The list of what's available on Linux and Apple here must be kept in sync
// with the logic in internal_make_render_window.cc.
bool ParseUseEglFromParams(const RenderEngineVtkParams& parameters) {
  const bool default_result = false;  // EGL is not (yet) the default.
  const std::string_view backend = parameters.backend;
  if (backend.empty()) {
    return default_result;
  }
  if (backend == "Cocoa") {
    if (kApple == false) {
      static const logging::Warn log_once(
          "RenderEngineVtkParams.backend = 'Cocoa' is not available");
      return default_result;
    }
    return default_result;
  }
  if (backend == "EGL") {
    if (kApple == true) {
      static const logging::Warn log_once(
          "RenderEngineVtkParams.backend = 'EGL' is not available");
      return default_result;
    }
    return true;
  }
  if (backend == "GLX") {
    if (kApple == true) {
      static const logging::Warn log_once(
          "RenderEngineVtkParams.backend = 'GLX' is not available");
      return default_result;
    }
    return false;
  }
  throw std::logic_error(fmt::format(
      "Unknown value for RenderEngineVtkParams.backend = '{}'; valid choices "
      "are: '' (empty), 'Cocoa', 'EGL', or 'GLX'",
      backend));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
