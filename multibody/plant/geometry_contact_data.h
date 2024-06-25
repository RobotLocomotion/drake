#pragma once

#include <vector>

#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"

namespace drake {
namespace multibody {
namespace internal {

/* This contains the geometric contact information coming out of SceneGraph, as
consumed by MultibodyPlant. Depending on MbP's contact model and the proximity
properties in the scene graph, one or the other vector might be guaranteed to be
empty; e.g., even in kPoint only mode we still have a field named `surfaces` but
it's always empty.

When T = Expression, the class is specialized to omit some member data, because
ContactSurface doesn't support Expression.

@tparam_default_scalar */
template <typename T>
struct GeometryContactData {
  std::vector<geometry::PenetrationAsPointPair<T>> point_pairs;
  std::vector<geometry::ContactSurface<T>> surfaces;
  // TODO(jwnimmer-tri) It seems to me like DeformableContact<T> should be
  // computed and cached here as well.
};

/* Full specialization of HydroelasticContactInfo for T = Expression. */
template <>
class GeometryContactData<symbolic::Expression> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryContactData);
  GeometryContactData() = default;

  using T = symbolic::Expression;
  std::vector<geometry::PenetrationAsPointPair<T>> point_pairs;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
