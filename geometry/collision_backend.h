#pragma once

#include <optional>
#include <string_view>

namespace drake {
namespace geometry {

/** The collision library used to answer SceneGraph proximity queries. */
enum class CollisionBackend {
  /** The Flexible Collision Library. */
  kFcl,
  /** The Coal collision library. */
  kCoal,
};

/** Returns the lowercase name of the collision backend. */
std::string_view to_string(CollisionBackend backend);

/** Parses a collision backend name, case insensitively. Returns nullopt when
`name` is not recognized. */
std::optional<CollisionBackend> ParseCollisionBackend(std::string_view name);

}  // namespace geometry
}  // namespace drake
