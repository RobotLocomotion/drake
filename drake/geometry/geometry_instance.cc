#include "drake/geometry/geometry_instance.h"

namespace drake {
namespace geometry {

// Explicitly instantiates on the most common scalar types.
template class GeometryInstance<double>;

}  // namespace geometry
}  // namespace drake
