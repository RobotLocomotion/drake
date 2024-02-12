#include "drake/geometry/shape_to_string.h"

namespace drake {
namespace geometry {

ShapeToString::~ShapeToString() = default;

void ShapeToString::DefaultImplementGeometry(const Shape& shape) {
  string_ = shape.to_string();
}

}  // namespace geometry
}  // namespace drake
