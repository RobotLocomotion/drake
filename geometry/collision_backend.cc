#include "drake/geometry/collision_backend.h"

#include <algorithm>
#include <cctype>
#include <string>

#include "drake/common/drake_assert.h"

namespace drake {
namespace geometry {

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

}  // namespace geometry
}  // namespace drake
