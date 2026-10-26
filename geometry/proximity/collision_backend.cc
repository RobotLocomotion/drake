#include "drake/geometry/proximity/collision_backend.h"

#include <cstdlib>
#include <optional>
#include <string_view>

#include "drake/common/text_logging.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

constexpr char kEnvName[] = "DRAKE_PROXIMITY_BACKEND";

/* The backend used when the environment says nothing useful. */
constexpr CollisionBackend kFallback = CollisionBackend::kFcl;

CollisionBackend ReadBackendFromEnvironment() {
  const char* const env_cstr = std::getenv(kEnvName);
  if (env_cstr == nullptr || *env_cstr == '\0') {
    return kFallback;
  }
  const std::string_view env_view{env_cstr};
  const std::optional<CollisionBackend> parsed =
      ParseCollisionBackend(env_view);
  if (!parsed.has_value()) {
    static const logging::Warn log_once(
        "The {} environment variable is set to '{}', which names no collision "
        "backend; using '{}'. The recognized values are '{}' and '{}'.",
        kEnvName, env_view, to_string(kFallback),
        to_string(CollisionBackend::kFcl), to_string(CollisionBackend::kCoal));
    return kFallback;
  }
  if (*parsed != kFallback) {
    static const logging::Warn log_once(
        "Using the '{}' collision backend, as requested by {}. This is not "
        "Drake's default; proximity query results will differ slightly from "
        "the documented values.",
        to_string(*parsed), kEnvName);
  }
  return *parsed;
}

}  // namespace

CollisionBackend GetDefaultCollisionBackend() {
  static const CollisionBackend value = ReadBackendFromEnvironment();
  return value;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
