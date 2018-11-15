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
  }
  DRAKE_ABORT_MSG("Unreachable code; switch missed a value");
}

std::ostream& operator<<(std::ostream& out, const Role& role) {
  out << to_string(role);
  return out;
}

Role convert_role(const geometry::Role& role) {
  switch (role) {
    case geometry::Role::kUnassigned:
      return Role::kUnassigned;
    case geometry::Role::kProximity:
      return Role::kProximity;
    case geometry::Role::kIllustration:
      return Role::kIllustration;
  }
  DRAKE_ABORT_MSG("Unreachable code; switch missed a value");
}

}  // namespace dev
}  // namespace geometry
}  // namespace drake
