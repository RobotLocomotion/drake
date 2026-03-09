#include "drake/perception/point_cloud_flags.h"

#include <vector>

#include "fmt/ranges.h"

namespace drake {
namespace perception {

namespace pc_flags {

std::ostream& operator<<(std::ostream& os, const Fields& fields) {
  return os << fmt::to_string(fields);
}

std::string to_string(const Fields& fields) {
  DRAKE_DEMAND(internal::kMaxBitInUse == kRGBs);
  std::vector<std::string> values;
  if (fields.contains(kXYZs)) {
    values.emplace_back("kXYZs");
  }
  if (fields.contains(kNormals)) {
    values.emplace_back("kNormals");
  }
  if (fields.contains(kRGBs)) {
    values.emplace_back("kRGBs");
  }
  if (fields.has_descriptor()) {
    values.emplace_back(fields.descriptor_type().name());
  }
  return fmt::format("({})", fmt::join(values, " | "));
}

}  // namespace pc_flags

}  // namespace perception
}  // namespace drake
