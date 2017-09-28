#include "drake/perception/point_cloud_flags.h"

#include <sstream>
#include <vector>

namespace drake {
namespace perception {

namespace {

// Utility for `ToString`.
std::ostream& join(std::ostream& os,
                   std::vector<std::string> elements,
                   const std::string& delim) {
  for (size_t i = 0; i < elements.size(); ++i) {
    os << elements[i];
    if (i + 1 < elements.size())
      os << delim;
  }
  return os;
}

}  // namespace

namespace pc_flags {

std::ostream& operator<<(std::ostream& os, const Fields& fields) {
  std::vector<std::string> values;
  if (fields.contains(pc_flags::kXYZs))
    values.push_back("kXYZs");
  if (fields.has_descriptor()) {
    values.push_back(fields.descriptor_type().name());
  }
  os << "(";
  join(os, values, " | ");
  return os << ")";
}

}  // namespace pc_flags

}  // namespace perception
}  // namespace drake
