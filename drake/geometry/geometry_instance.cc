#include "drake/geometry/geometry_instance.h"

#include <utility>

namespace drake {
namespace geometry {

template <typename T>
GeometryInstance<T>::GeometryInstance(const Isometry3<T>& X_PG,
                                      std::unique_ptr<Shape> shape)
    : X_FG_(X_PG), shape_(std::move(shape)) {}

// Explicitly instantiates on the most common scalar types.
template class GeometryInstance<double>;

}  // namespace geometry
}  // namespace drake
