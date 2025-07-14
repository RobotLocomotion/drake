#pragma once

#include <optional>

#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

/** Calculates the oriented bounding box (OBB) for the Shape in its canonical
 frame. Returns `std::nullopt` if the Shape is HalfSpace which doesn't have a
 bounding box.

 @throws std::exception if a referenced file cannot be opened. */
std::optional<Obb> CalcObb(const Shape& shape);

}  // namespace geometry
}  // namespace drake
