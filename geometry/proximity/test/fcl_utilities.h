#pragma once

#include <memory>

#include <fcl/fcl.h>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

// Creates an fcl::CollisionObjectd for a given Drake Shape, stamping the
// geometry id into the FCL object's user data (as required by
// the various Callback types).
std::unique_ptr<fcl::CollisionObjectd> MakeFclObject(const Shape& shape,
                                                     GeometryId id,
                                                     bool is_dynamic);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
