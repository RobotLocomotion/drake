#include "drake/geometry/geometry_roles.h"

#include <string>

namespace drake {
namespace geometry {

std::string to_string(const Role& role) {
  switch (role) {
    case Role::kProximity:
      return "proximity";
    case Role::kIllustration:
      return "illustration";
    case Role::kUnassigned:
      return "unassigned";
  }
  return "unknown";
}

std::ostream& operator<<(std::ostream& out, const Role& role) {
  out << to_string(role);
  return out;
}

}  // namespace geometry
}  // namespace drake
