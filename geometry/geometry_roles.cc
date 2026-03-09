#include "drake/geometry/geometry_roles.h"

#include <string>

namespace drake {
namespace geometry {

ProximityProperties::ProximityProperties(const GeometryProperties& other)
    : GeometryProperties(other) {}

ProximityProperties::~ProximityProperties() = default;

PerceptionProperties::PerceptionProperties(const GeometryProperties& other)
    : GeometryProperties(other) {}

PerceptionProperties::~PerceptionProperties() = default;

IllustrationProperties::IllustrationProperties(const GeometryProperties& other)
    : GeometryProperties(other) {}

IllustrationProperties::~IllustrationProperties() = default;

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
  return "unknown";
}

IllustrationProperties MakePhongIllustrationProperties(
    const Vector4<double>& diffuse) {
  IllustrationProperties props;
  props.AddProperty("phong", "diffuse", diffuse);
  return props;
}

}  // namespace geometry
}  // namespace drake
