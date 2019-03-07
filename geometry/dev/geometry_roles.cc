#include "drake/geometry/dev/geometry_roles.h"

#include <string>

namespace drake {
namespace geometry {
namespace dev {

std::string to_string(const Role& role) {
  switch (role) {
    case Role::kProximity:
      return "proximity";
    case Role::kPerception:
      return "perception";
    case Role::kIllustration:
      return "illustration";
    case Role::kUnassigned:
      return "unassigned";
    default:
      return "unknown";
  }
}

std::ostream& operator<<(std::ostream& out, const Role& role) {
  out << to_string(role);
  return out;
}

}  // namespace dev
}  // namespace geometry
}  // namespace drake
