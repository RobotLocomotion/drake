#include "drake/geometry/proximity/sorted_triplet.h"

namespace drake {
namespace geometry {
namespace internal {

// Some template instantiations.
template struct SortedTriplet<double>;
template struct SortedTriplet<int>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
