#pragma once

#include <string>

#include "drake/geometry/geometry_roles.h"

namespace drake {
namespace geometry {
namespace dev {

/** The set of properties for geometry used in a "perception" role.

 Examples of functionality that depends on the perception role:
   - n/a
 */
class PerceptionProperties final : public GeometryProperties{
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PerceptionProperties);
  // TODO(SeanCurtis-TRI): Should this have a render label built in?

  // TODO(SeanCurtis-TRI): Consider adding PerceptionIndex to this.
  PerceptionProperties() = default;
};

/** General enumeration for indicating geometry role.  */
enum class Role {
  kUnassigned = 0x0,
  kProximity = 0x1,
  kIllustration = 0x2,
  kPerception = 0x4
};

/** @name  Geometry role to string conversions.

 These are simply convenience functions for converting the Role enumeration into
 a human-readable string. */
//@{

std::string to_string(const Role& role);

std::ostream& operator<<(std::ostream& out, const Role& role);

Role convert_role(const geometry::Role& role);

//@}

}  // namespace dev
}  // namespace geometry
}  // namespace drake
