#include "drake/geometry/proximity/collision_backend.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdlib>
#include <optional>
#include <string>
#include <string_view>

#include "drake/common/drake_assert.h"
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

/* Reading the environment once, on first use, is what makes this safe to call
 from any thread: the function-local static's initialization is
 thread-safe, and the atomic covers the setter racing a reader. */
std::atomic<CollisionBackend>& backend_storage() {
  static std::atomic<CollisionBackend> value{ReadBackendFromEnvironment()};
  return value;
}

}  // namespace

std::string_view to_string(CollisionBackend backend) {
  switch (backend) {
    case CollisionBackend::kFcl:
      return "fcl";
    case CollisionBackend::kCoal:
      return "coal";
  }
  DRAKE_UNREACHABLE();
}

std::optional<CollisionBackend> ParseCollisionBackend(std::string_view name) {
  std::string lower(name);
  std::transform(lower.begin(), lower.end(), lower.begin(), [](char ch) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  });
  if (lower == "fcl") return CollisionBackend::kFcl;
  if (lower == "coal") return CollisionBackend::kCoal;
  return std::nullopt;
}

CollisionBackend GetDefaultCollisionBackend() {
  return backend_storage().load(std::memory_order_relaxed);
}

void SetDefaultCollisionBackend(CollisionBackend backend) {
  backend_storage().store(backend, std::memory_order_relaxed);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
