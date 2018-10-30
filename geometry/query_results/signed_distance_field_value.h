#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

template <typename T>
struct SignedDistanceFieldValue{
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SignedDistanceFieldValue)

  SignedDistanceFieldValue() = default;

  SignedDistanceFieldValue(GeometryId a, const Vector3<T>& near, T dist,
                           const Vector3<T>& grad)
  : anchored_id(a), nearest_point(near), distance(dist), gradient(grad) {}

  GeometryId anchored_id;
  Vector3<T> nearest_point;
  T distance{};
  Vector3<T> gradient;
};

}  // namespace geometry
}  // namespace drake
