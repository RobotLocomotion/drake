#include "drake/geometry/proximity/hydroelastic_type.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace geometry {
namespace internal {

std::ostream& operator<<(std::ostream& out, const HydroelasticType& type) {
  switch (type) {
    case HydroelasticType::kUndefined:
      out << "undefined";
      break;
    case HydroelasticType::kRigid:
      out << "rigid";
      break;
    case HydroelasticType::kSoft:
      out << "soft";
      break;
    default:
      DRAKE_UNREACHABLE();
  }
  return out;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
