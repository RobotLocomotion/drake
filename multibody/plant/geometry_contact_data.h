#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/deformable_contact.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"

namespace drake {
namespace multibody {
namespace internal {

/* This contains the geometric contact information coming out of SceneGraph, as
consumed by MultibodyPlant. Depending on MbP's contact model and the proximity
properties in the scene graph, one or the other vector might be guaranteed to be
empty; e.g., even in kPoint only mode we still have a field named `surfaces` but
it's always empty.

When T != double, the class is specialized to omit member data that is
incompatible with such scalars.

This class is almost never used directly, rather via the GeometryContactData
reference-counted wrapper.

@tparam_default_scalar */
template <typename T>
struct NestedGeometryContactData {
  std::vector<geometry::PenetrationAsPointPair<T>> point_pairs;
  std::vector<geometry::ContactSurface<T>> surfaces;
  geometry::internal::DeformableContact<T> deformable;
};

/* Full specialization of NestedGeometryContactData for T = AutoDiffXd.
This omits DeformableContact data. */
template <>
struct NestedGeometryContactData<AutoDiffXd> {
  using T = AutoDiffXd;
  std::vector<geometry::PenetrationAsPointPair<T>> point_pairs;
  std::vector<geometry::ContactSurface<T>> surfaces;
};

/* Full specialization of NestedGeometryContactData for T = Expression.
This omits ContactSurface and DeformableContact data. */
template <>
struct NestedGeometryContactData<symbolic::Expression> {
  using T = symbolic::Expression;
  std::vector<geometry::PenetrationAsPointPair<T>> point_pairs;
};

/* A reference-counted, immutable wrapper around NestedGeometryContactData, for
use with AbstractValue type erasure.

@tparam_default_scalar */
template <typename T>
class GeometryContactData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryContactData);
  GeometryContactData() = default;

  /* Returns a reference to the nested data. If there isn't any nested data,
  a completely empty nested object will be returned. */
  const NestedGeometryContactData<T>& get() const {
    return (data_ != nullptr) ? *data_ : get_empty_ref();
  }

  /* Set this object back to its default-constructed, empty state. */
  void Clear();

  /* Resets this to fresh storage and returns a mutable reference to it. */
  NestedGeometryContactData<T>& Allocate();

  /* Returns a shared_ptr to the data. Note that this aliases the *current* data
  storage. Future calls that Allocate() and then mutate have no effect on this
  return value. */
  std::shared_ptr<const NestedGeometryContactData<T>> Share() const;

 private:
  /* Returns a forever-valid reference (to static storage) that is empty. */
  static const NestedGeometryContactData<T>& get_empty_ref();

  std::shared_ptr<const NestedGeometryContactData<T>> data_{nullptr};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::multibody::internal::NestedGeometryContactData);

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::GeometryContactData);
