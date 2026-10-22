#pragma once

#include <memory>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

// Creates a collision object for a given Drake Shape, stamping the geometry id
// into the object's user data (as required by the various Callback types) and
// setting the object's pose to the given pose. `Backend` is a backend policy;
// see fcl_backend.h.
template <typename Backend>
std::unique_ptr<typename Backend::Object> MakeBackendObject(
    const Shape& shape, GeometryId id, bool is_dynamic,
    const math::RigidTransformd& X_WG = math::RigidTransformd::Identity());

}  // namespace internal
}  // namespace geometry
}  // namespace drake
