#include "drake/geometry/geometry_properties.h"

namespace drake {
namespace geometry {

std::ostream& operator<<(std::ostream& out, const GeometryProperties& props) {
  int i = 0;
  for (const auto& group_pair : props.values_) {
    const std::string& group_name = group_pair.first;
    const GeometryProperties::Group& group_properties =
        group_pair.second;
    out << "[" << group_name << "]";
    for (const auto& property_pair : group_properties) {
      const std::string& property_name = property_pair.first;
      out << "\n  " << property_name << ": "
          << property_pair.second->GetNiceTypeName();
      // TODO(SeanCurtis-TRI): How do I print the value in an AbstractValue?
    }
    if (i < props.num_groups() - 1) out << "\n";
    ++i;
  }
  return out;
}

}  // namespace geometry
}  // namespace drake
