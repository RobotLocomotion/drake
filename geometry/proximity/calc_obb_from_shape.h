#pragma once

#include <optional>

#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

/** Calculates the oriented bounding box (OBB) for the Shape. If a shape does
 not support OBB computation, this function returns `std::nullopt`.

 @throws std::exception if a referenced file cannot be opened.
*/
std::optional<Obb> CalcObb(const Shape& shape);

}  // namespace geometry
}  // namespace drake
