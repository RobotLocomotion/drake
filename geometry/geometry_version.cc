#include "drake/geometry/geometry_version.h"

namespace drake {
namespace geometry {

GeometryVersion::GeometryVersion()
    : proximity_version_id_(RoleVersionId::get_new_id()),
      perception_version_id_(RoleVersionId::get_new_id()),
      illustration_version_id_(RoleVersionId::get_new_id()) {}

bool GeometryVersion::IsSameAs(const GeometryVersion& other, Role role) const {
  switch (role) {
    case Role::kUnassigned:
      throw std::logic_error(
          "Trying to compare the version of unassigned roles.");
    case Role::kProximity:
      return proximity_version_id_ == other.proximity_version_id_;
    case Role::kIllustration:
      return illustration_version_id_ == other.illustration_version_id_;
    case Role::kPerception:
      return perception_version_id_ == other.perception_version_id_;
  }
  DRAKE_UNREACHABLE();
}

void GeometryVersion::modify_proximity() {
  proximity_version_id_ = RoleVersionId::get_new_id();
}

void GeometryVersion::modify_perception() {
  perception_version_id_ = RoleVersionId::get_new_id();
}

void GeometryVersion::modify_illustration() {
  illustration_version_id_ = RoleVersionId::get_new_id();
}
}  // namespace geometry
}  // namespace drake
