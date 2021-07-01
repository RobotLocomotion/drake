#include "drake/geometry/optimization/convex_set.h"

#include <memory>

namespace drake {
namespace geometry {
namespace optimization {

ConvexSet::ConvexSet(
    std::function<std::unique_ptr<ConvexSet>(const ConvexSet&)> cloner,
    int ambient_dimension)
    : cloner_(std::move(cloner)), ambient_dimension_(ambient_dimension) {
  DRAKE_DEMAND(ambient_dimension >= 0);
}

std::unique_ptr<ConvexSet> ConvexSet::Clone() const { return cloner_(*this); }

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
